#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zmq.h>

#include <tagtracker_wireformat/zmq_iq_packet.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double kTotalDecimation = 8.0 * 5.0 * 5.0;
constexpr std::size_t kBytesPerIQ = TTWF_ZMQ_IQ_BYTES_PER_COMPLEX_SAMPLE;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr uint32_t kZmqMagic = TTWF_ZMQ_IQ_MAGIC;
constexpr uint16_t kZmqVersion = TTWF_ZMQ_IQ_VERSION;
constexpr uint16_t kZmqHeaderSizeBytes = TTWF_ZMQ_IQ_HEADER_SIZE;

struct Options {
    double inputRate = 0.0;
    bool strictInputRate = false;
    std::size_t packetSamples = 1024;
    std::string zmqEndpoint = "tcp://127.0.0.1:5555";
    double rateTolerancePpm = 5000.0;
    std::string ip = "127.0.0.1";
    std::vector<uint16_t> ports = {10000, 10001};
    double shiftKhz = 10.0;
};

struct ArgsError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

void printUsage(const char *argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --input-rate <Hz>     Expected incoming IQ sample rate; 0 "
                 "means auto-learn from first packet (default 0)\n"
              << "  --strict-input-rate   Hard-fail on sample-rate mismatch "
                 "beyond --rate-tol-ppm\n"
              << "  --shift-khz <kHz>     Frequency shift before decimation: "
                 "positive shifts up, negative shifts down (default 10)\n"
              << "  --frame <samples>     Total complex samples per UDP "
                 "packet, including the timestamp (default 1024)\n"
              << "  --zmq-endpoint <uri>  ZeroMQ SUB endpoint (default "
                 "tcp://127.0.0.1:5555)\n"
              << "  --rate-tol-ppm <ppm>  Allowed sample-rate error before "
                 "warning (default 5000)\n"
              << "  --ip <addr>           Destination IPv4 address (default "
                 "127.0.0.1)\n"
              << "  --ports <p0,p1,...>   Comma-separated UDP ports (default "
                 "10000,10001)\n"
              << "  --help                Show this message\n";
}

Options parseArgs(int argc, char **argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string_view arg(argv[i]);
        if (arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--input-rate") {
            if (++i >= argc) {
                throw ArgsError("--input-rate requires a value");
            }
            opts.inputRate = std::stod(argv[i]);
        } else if (arg == "--strict-input-rate") {
            opts.strictInputRate = true;
        } else if (arg == "--frame") {
            if (++i >= argc) {
                throw ArgsError("--frame requires a value");
            }
            opts.packetSamples = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--zmq-endpoint") {
            if (++i >= argc) {
                throw ArgsError("--zmq-endpoint requires a value");
            }
            opts.zmqEndpoint = argv[i];
        } else if (arg == "--rate-tol-ppm") {
            if (++i >= argc) {
                throw ArgsError("--rate-tol-ppm requires a value");
            }
            opts.rateTolerancePpm = std::stod(argv[i]);
        } else if (arg == "--ip") {
            if (++i >= argc) {
                throw ArgsError("--ip requires a value");
            }
            opts.ip = argv[i];
        } else if (arg == "--shift-khz") {
            if (++i >= argc) {
                throw ArgsError("--shift-khz requires a value");
            }
            opts.shiftKhz = std::stod(argv[i]);
        } else if (arg == "--ports") {
            if (++i >= argc) {
                throw ArgsError("--ports requires a value");
            }
            opts.ports.clear();
            std::string value(argv[i]);
            std::size_t start = 0;
            while (start < value.size()) {
                std::size_t comma = value.find(',', start);
                auto token = value.substr(start, comma == std::string::npos
                                                     ? std::string::npos
                                                     : comma - start);
                if (!token.empty()) {
                        const unsigned long parsedPort = std::stoul(token);
                        if (parsedPort == 0UL || parsedPort > 65535UL) {
                            throw ArgsError("--ports values must be in range 1..65535");
                        }
                        opts.ports.push_back(static_cast<uint16_t>(parsedPort));
                }
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
            if (opts.ports.empty()) {
                throw ArgsError("--ports requires at least one port number");
            }
        } else {
            throw ArgsError("Unknown option: " + std::string(arg));
        }
    }
    if (opts.inputRate < 0.0) {
        throw ArgsError("input-rate must be >= 0 (0 enables auto-learn)");
    }
    if (opts.packetSamples < 2) {
        throw ArgsError(
            "frame must be at least 2 samples (timestamp + payload)");
    }
    if (opts.zmqEndpoint.empty()) {
        throw ArgsError("zmq-endpoint must not be empty");
    }
    if (opts.rateTolerancePpm <= 0.0) {
        throw ArgsError("rate-tol-ppm must be positive");
    }
    return opts;
}

std::vector<float> designLowpass(std::size_t taps, float cutoff) {
    if (taps < 3) {
        taps = 3;
    }
    if ((taps % 2) == 0) {
        ++taps;
    }
    constexpr float pi = 3.14159265358979323846f;
    std::vector<float> coeffs(taps);
    const float M = static_cast<float>(taps - 1);
    for (std::size_t n = 0; n < taps; ++n) {
        const float m = static_cast<float>(n) - M / 2.0f;
        const float window =
            0.54f - 0.46f * std::cos(2.0f * pi * static_cast<float>(n) / M);
        float sinc = 0.0f;
        if (std::abs(m) < 1e-6f) {
            sinc = 2.0f * cutoff;
        } else {
            sinc = std::sin(2.0f * pi * cutoff * m) / (pi * m);
        }
        coeffs[n] = window * sinc;
    }
    float sum = 0.0f;
    for (float c : coeffs) {
        sum += c;
    }
    if (sum != 0.0f) {
        for (float &c : coeffs) {
            c /= sum;
        }
    }
    return coeffs;
}

class FirDecimator {
  public:
    FirDecimator(int factor, std::size_t taps, float cutoff)
        : factor_(factor), taps_(designLowpass(taps, cutoff)),
          state_(taps_.size()) {}

    std::vector<std::complex<float>>
    process(const std::vector<std::complex<float>> &input) {
        std::vector<std::complex<float>> output;
        if (factor_ <= 0 || taps_.empty()) {
            return output;
        }
        output.reserve(input.size() / factor_ + 1);
        for (const auto &sample : input) {
            state_[writeIndex_] = sample;
            writeIndex_ = (writeIndex_ + 1) % state_.size();
            phase_ = (phase_ + 1) % factor_;
            if (phase_ == 0) {
                std::complex<float> acc{0.0f, 0.0f};
                std::size_t idx = writeIndex_;
                for (std::size_t k = 0; k < taps_.size(); ++k) {
                    idx = (idx == 0) ? state_.size() - 1 : idx - 1;
                    acc += state_[idx] * taps_[k];
                }
                output.push_back(acc);
            }
        }
        return output;
    }

  private:
    int factor_;
    std::vector<float> taps_;
    std::vector<std::complex<float>> state_;
    std::size_t writeIndex_ = 0;
    int phase_ = 0;
};

class FrequencyShifter {
  public:
    FrequencyShifter(double sampleRate, double shiftHz) : shiftHz_(shiftHz) {
        if (sampleRate <= 0.0) {
            sampleRate = 1.0;
        }
        step_ = (shiftHz_ == 0.0) ? 0.0 : (kTwoPi * shiftHz_ / sampleRate);
    }

    void mix(std::vector<std::complex<float>> &samples) {
        if (shiftHz_ == 0.0 || samples.empty()) {
            return;
        }
        for (auto &sample : samples) {
            const auto c = static_cast<float>(std::cos(phase_));
            const auto s = static_cast<float>(std::sin(phase_));
            sample *= std::complex<float>(c, s);
            phase_ += step_;
            if (phase_ > kPi) {
                phase_ -= kTwoPi;
            } else if (phase_ < -kPi) {
                phase_ += kTwoPi;
            }
        }
    }

  private:
    double shiftHz_ = 0.0;
    double step_ = 0.0;
    double phase_ = 0.0;
};

class TimestampEncoder {
  public:
    explicit TimestampEncoder(double sampleRate) : sampleRate_(sampleRate) {
        if (sampleRate_ > 0.0) {
            const double rounded = std::round(sampleRate_);
            if (std::abs(sampleRate_ - rounded) < 1e-6) {
                sampleRateHz_ = static_cast<uint64_t>(rounded);
            }
        }
    }

    void reset() { anchored_ = false; }

    std::complex<float> headerForSample(uint64_t sampleIndex) {
        if (!anchored_) {
            anchorToWallClock();
        }
        uint32_t seconds = 0;
        uint32_t nanoseconds = 0;

        if (sampleRateHz_ > 0) {
            const unsigned __int128 nsNumerator =
                static_cast<unsigned __int128>(sampleIndex) * 1000000000ULL;
            const uint64_t nsOffset =
                static_cast<uint64_t>(nsNumerator / sampleRateHz_);
            const uint64_t nsecTotal =
                static_cast<uint64_t>(baseNsec_) + nsOffset;

            seconds =
                baseSec_ + static_cast<uint32_t>(nsecTotal / 1000000000ULL);
            nanoseconds = static_cast<uint32_t>(nsecTotal % 1000000000ULL);
        } else {
            double absolute =
                baseSeconds_ + static_cast<double>(sampleIndex) / sampleRate_;
            seconds = static_cast<uint32_t>(absolute);
            double fractional = absolute - static_cast<double>(seconds);
            nanoseconds =
                static_cast<uint32_t>(std::round(fractional * 1'000'000'000.0));
            if (nanoseconds >= 1'000'000'000U) {
                nanoseconds -= 1'000'000'000U;
                ++seconds;
            }
        }

        float secBits = 0.0f;
        float nsecBits = 0.0f;
        std::memcpy(&secBits, &seconds, sizeof(uint32_t));
        std::memcpy(&nsecBits, &nanoseconds, sizeof(uint32_t));
        return {secBits, nsecBits};
    }

  private:
    void anchorToWallClock() {
        timespec ts{};
        clock_gettime(CLOCK_REALTIME, &ts);
        baseSec_ = static_cast<uint32_t>(ts.tv_sec);
        baseNsec_ = static_cast<uint32_t>(ts.tv_nsec);
        baseSeconds_ = static_cast<double>(ts.tv_sec) +
                       static_cast<double>(ts.tv_nsec) * 1e-9;
        anchored_ = true;
    }

    double sampleRate_;
    uint64_t sampleRateHz_ = 0;
    uint32_t baseSec_ = 0;
    uint32_t baseNsec_ = 0;
    double baseSeconds_ = 0.0;
    bool anchored_ = false;
};

class UdpStreamer {
  public:
    UdpStreamer(std::string ip, const std::vector<uint16_t> &ports) {
        sockaddr_in templateAddr{};
        templateAddr.sin_family = AF_INET;
        if (inet_pton(AF_INET, ip.c_str(), &templateAddr.sin_addr) != 1) {
            throw std::runtime_error("Invalid IPv4 address");
        }
        for (auto port : ports) {
            if (port == 0) {
                continue;
            }
            int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) {
                throw std::runtime_error("Failed to create UDP socket");
            }
            sockaddr_in addr = templateAddr;
            addr.sin_port = htons(port);
            sockets_.push_back({fd, addr});
        }
        if (sockets_.empty()) {
            throw std::runtime_error("No valid UDP ports configured");
        }
    }

    ~UdpStreamer() {
        for (const auto &socket : sockets_) {
            if (socket.fd >= 0) {
                ::close(socket.fd);
            }
        }
    }

    void send(const std::vector<std::complex<float>> &frame) const {
        const auto *raw = reinterpret_cast<const char *>(frame.data());
        const std::size_t bytes = frame.size() * sizeof(std::complex<float>);
        ++packetsSent_;
        if (packetsSent_ == 1 || (packetsSent_ % 500) == 0) {
            std::cerr << "airspyhf_decimator: sent packets=" << packetsSent_
                      << " send_errors=" << sendErrors_ << "\n";
        }
        for (const auto &socket : sockets_) {
            ssize_t sent =
                ::sendto(socket.fd, raw, bytes, 0,
                         reinterpret_cast<const sockaddr *>(&socket.addr),
                         sizeof(sockaddr_in));
            if (sent < 0) {
                ++sendErrors_;
                std::perror("sendto");
                if (sendErrors_ == 1 || (sendErrors_ % 100) == 0) {
                    std::cerr << "UDP send failures: " << sendErrors_ << " of "
                              << packetsSent_ << " packets\n";
                }
            } else if (static_cast<std::size_t>(sent) != bytes) {
                ++sendErrors_;
                std::cerr << "Partial UDP send: sent " << sent
                          << " bytes, expected " << bytes << "\n";
            }
        }
    }

  private:
    struct SocketSlot {
        int fd;
        sockaddr_in addr;
    };

    std::vector<SocketSlot> sockets_;
    mutable uint64_t packetsSent_ = 0;
    mutable uint64_t sendErrors_ = 0;
};

std::vector<std::complex<float>> convertToComplex(const uint8_t *bytes,
                                                  std::size_t size) {
    if ((bytes == nullptr && size != 0U) || (size % kBytesPerIQ) != 0U) {
        throw std::runtime_error("Unaligned IQ byte stream");
    }
    std::vector<std::complex<float>> result;
    result.reserve(size / kBytesPerIQ);
    for (std::size_t offset = 0; offset + kBytesPerIQ <= size;
         offset += kBytesPerIQ) {
        float i = 0.0f;
        float q = 0.0f;
        std::memcpy(&i, bytes + offset, sizeof(float));
        std::memcpy(&q, bytes + offset + sizeof(float), sizeof(float));
        result.emplace_back(i, q);
    }
    return result;
}

struct ZmqPacket {
    uint64_t sequence = 0;
    uint64_t timestampUs = 0;
    uint32_t sampleRate = 0;
    uint32_t sampleCount = 0;
    uint32_t flags = 0;
    uint32_t payloadBytes = 0;
    std::vector<std::complex<float>> samples;
};

bool parseZmqFrame(const std::vector<uint8_t> &frame, ZmqPacket &packet) {
    ttwf_zmq_iq_packet_header_t header{};
    const int validateRc =
        ttwf_validate_zmq_iq_frame(frame.data(), frame.size(), &header);
    if (validateRc != TTWF_ZMQ_OK) {
        return false;
    }

    const std::size_t headerSize = static_cast<std::size_t>(header.header_size);
    const std::size_t payloadBytes =
        static_cast<std::size_t>(header.payload_bytes);

    packet.sequence = header.sequence;
    packet.timestampUs = header.timestamp_us;
    packet.sampleRate = header.sample_rate;
    packet.sampleCount = header.sample_count;
    packet.flags = header.flags;
    packet.payloadBytes = header.payload_bytes;
    packet.samples =
        convertToComplex(frame.data() + headerSize, payloadBytes);
    return true;
}

class ZmqIqReceiver {
  public:
        ZmqIqReceiver(const ZmqIqReceiver &) = delete;
        ZmqIqReceiver &operator=(const ZmqIqReceiver &) = delete;
        ZmqIqReceiver(ZmqIqReceiver &&) = delete;
        ZmqIqReceiver &operator=(ZmqIqReceiver &&) = delete;

    explicit ZmqIqReceiver(const std::string &endpoint)
        : context_(zmq_ctx_new()) {
        if (context_ == nullptr) {
            throw std::runtime_error("Failed to create ZeroMQ context");
        }
        socket_ = zmq_socket(context_, ZMQ_SUB);
        if (socket_ == nullptr) {
            cleanup();
            throw std::runtime_error("Failed to create ZeroMQ SUB socket");
        }

        const int rcvTimeoutMs = 1000;
        if (zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &rcvTimeoutMs,
                           sizeof(rcvTimeoutMs)) != 0) {
            cleanup();
            throw std::runtime_error("Failed to set ZeroMQ receive timeout");
        }

        const char *allTopicsFilter = "";
        if (zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, allTopicsFilter, 0) != 0) {
            cleanup();
            throw std::runtime_error("Failed to subscribe ZeroMQ socket");
        }

        if (zmq_connect(socket_, endpoint.c_str()) != 0) {
            cleanup();
            throw std::runtime_error("Failed to connect ZeroMQ endpoint: " +
                                     endpoint);
        }
    }

    ~ZmqIqReceiver() {
        cleanup();
    }

    bool receive(ZmqPacket &packet, bool &timedOut) {
        timedOut = false;

        std::vector<uint8_t> firstFrame;
        bool hasMore = false;
        if (!receiveFrame(firstFrame, hasMore, timedOut)) {
            return false;
        }

        std::vector<std::vector<uint8_t>> parts;
        parts.push_back(std::move(firstFrame));

        while (hasMore) {
            std::vector<uint8_t> nextFrame;
            if (!receiveFrame(nextFrame, hasMore, timedOut)) {
                ++malformedPackets_;
                timedOut = false;
                return false;
            }
            parts.push_back(std::move(nextFrame));
        }

        if (parts.size() == 1U) {
            if (tryParseFrame(parts.front(), packet)) {
                return true;
            }
            ++malformedPackets_;
            return false;
        }

        std::size_t totalBytes = 0;
        for (const auto &part : parts) {
            totalBytes += part.size();
        }

        std::vector<uint8_t> combined;
        combined.reserve(totalBytes);
        for (const auto &part : parts) {
            combined.insert(combined.end(), part.begin(), part.end());
        }

        if (tryParseFrame(combined, packet)) {
            return true;
        }

        for (const auto &part : parts) {
            if (tryParseFrame(part, packet)) {
                return true;
            }
        }

        ++malformedPackets_;
        return false;
    }

    uint64_t malformedPackets() const { return malformedPackets_; }

  private:
    void cleanup() {
        if (socket_ != nullptr) {
            zmq_close(socket_);
            socket_ = nullptr;
        }
        if (context_ != nullptr) {
            zmq_ctx_term(context_);
            context_ = nullptr;
        }
    }
    bool receiveFrame(std::vector<uint8_t> &frame, bool &hasMore,
                      bool &timedOut) {
        frame.clear();
        timedOut = false;

        zmq_msg_t msg;
        zmq_msg_init(&msg);
        const int recvRc = zmq_msg_recv(&msg, socket_, 0);
        if (recvRc < 0) {
            zmq_msg_close(&msg);
            if (errno == EAGAIN) {
                timedOut = true;
                return false;
            }
            throw std::runtime_error("ZeroMQ receive failed");
        }

        const std::size_t messageSize = zmq_msg_size(&msg);
        frame.resize(messageSize);
        if (messageSize > 0U) {
            std::memcpy(frame.data(), zmq_msg_data(&msg), messageSize);
        }

        int moreValue = 0;
        std::size_t moreSize = sizeof(moreValue);
        hasMore = false;
        if (zmq_getsockopt(socket_, ZMQ_RCVMORE, &moreValue, &moreSize) == 0) {
            hasMore = (moreValue != 0);
        }

        zmq_msg_close(&msg);
        return true;
    }

    bool tryParseFrame(const std::vector<uint8_t> &frame, ZmqPacket &packet) {
        return parseZmqFrame(frame, packet);
    }

    void *context_ = nullptr;
    void *socket_ = nullptr;
    uint64_t malformedPackets_ = 0;
};

volatile std::sig_atomic_t gShouldStop = 0;

void handleTerminationSignal(int) { gShouldStop = 1; }

} // namespace

int main(int argc, char **argv) {
    try {
        std::signal(SIGINT, handleTerminationSignal);
        std::signal(SIGTERM, handleTerminationSignal);
        std::signal(SIGPIPE, SIG_IGN);
        auto opts = parseArgs(argc, argv);

        std::cerr << "airspyhf_decimator: zmq=" << opts.zmqEndpoint
                  << " inputRateExpected=" << opts.inputRate
                  << " strictInputRate="
                  << (opts.strictInputRate ? "true" : "false")
                  << " shiftKhz=" << opts.shiftKhz
                  << " frame=" << opts.packetSamples
                  << " rateTolPpm=" << opts.rateTolerancePpm << "\n";

        FirDecimator stage1(8, 8 * 16, 0.45f / 8.0f);
        FirDecimator stage2(5, 5 * 16, 0.45f / 5.0f);
        FirDecimator stage3(5, 5 * 16, 0.45f / 5.0f);

        ZmqIqReceiver receiver(opts.zmqEndpoint);
        std::unique_ptr<TimestampEncoder> timestampEncoder;
        UdpStreamer streamer(opts.ip, opts.ports);
        std::unique_ptr<FrequencyShifter> frequencyShifter;

        const std::size_t payloadSamples = opts.packetSamples - 1;

        std::vector<std::complex<float>> buffer;
        buffer.reserve(payloadSamples * 2);

        uint64_t samplesSent = 0;
        uint64_t inputSamplesProcessed = 0;
        uint64_t outputSamplesProduced = 0;
        uint64_t framesSent = 0;
        uint64_t zmqBytesRead = 0;
        uint64_t zmqPacketsReceived = 0;
        uint64_t droppedPackets = 0;
        uint64_t outOfOrderPackets = 0;
        uint64_t sampleRateFieldWarnings = 0;
        uint64_t measuredRateWarnings = 0;
        uint64_t prevSequence = 0;
        bool haveSequence = false;
        uint64_t firstZmqTimestampUs = 0;
        uint64_t lastZmqTimestampUs = 0;
        double effectiveInputRate = 0.0;
        double effectiveOutputRate = 0.0;

        auto runStart = std::chrono::steady_clock::now();
        auto lastPerfLog = runStart;
        std::chrono::steady_clock::duration processingTime{};

        while (gShouldStop == 0) {
            ZmqPacket packet;
            bool timedOut = false;
            if (!receiver.receive(packet, timedOut)) {
                if (timedOut) {
                    continue;
                }
                if (receiver.malformedPackets() == 1 ||
                    (receiver.malformedPackets() % 100) == 0) {
                    std::cerr << "airspyhf_decimator: malformed ZMQ packets="
                              << receiver.malformedPackets() << "\n";
                }
                continue;
            }
            ++zmqPacketsReceived;
            zmqBytesRead += static_cast<uint64_t>(kZmqHeaderSizeBytes +
                                                  packet.payloadBytes);

            if (firstZmqTimestampUs == 0) {
                firstZmqTimestampUs = packet.timestampUs;
            }
            lastZmqTimestampUs = packet.timestampUs;

            if (!haveSequence) {
                haveSequence = true;
            } else if (packet.sequence > (prevSequence + 1)) {
                const uint64_t newlyDropped =
                    packet.sequence - (prevSequence + 1);
                droppedPackets += newlyDropped;
                std::cerr << "airspyhf_decimator: dropped " << newlyDropped
                          << " packet(s) before sequence=" << packet.sequence
                          << " total_dropped=" << droppedPackets << "\n";
            } else if (packet.sequence <= prevSequence) {
                ++outOfOrderPackets;
                std::cerr << "airspyhf_decimator: out-of-order/duplicate "
                             "packet sequence="
                          << packet.sequence << " previous=" << prevSequence
                          << " count=" << outOfOrderPackets << "\n";
            }
            prevSequence = packet.sequence;

            if (effectiveInputRate <= 0.0) {
                effectiveInputRate =
                    (opts.inputRate > 0.0)
                        ? opts.inputRate
                        : static_cast<double>(packet.sampleRate);
                const double firstPacketRateErrorPpm =
                    (effectiveInputRate > 0.0)
                        ? (1e6 *
                           std::abs(static_cast<double>(packet.sampleRate) -
                                    effectiveInputRate) /
                           effectiveInputRate)
                        : 0.0;
                if (firstPacketRateErrorPpm > opts.rateTolerancePpm) {
                    if (opts.strictInputRate) {
                        throw std::runtime_error(
                            "Strict input-rate mismatch on first packet");
                    }
                    ++sampleRateFieldWarnings;
                    std::cerr
                        << "airspyhf_decimator: bad incoming sample_rate field="
                        << packet.sampleRate
                        << " expected=" << effectiveInputRate
                        << " error_ppm=" << firstPacketRateErrorPpm
                        << " warnings=" << sampleRateFieldWarnings << "\n";
                }

                effectiveOutputRate = effectiveInputRate / kTotalDecimation;
                timestampEncoder =
                    std::make_unique<TimestampEncoder>(effectiveOutputRate);
                frequencyShifter = std::make_unique<FrequencyShifter>(
                    effectiveInputRate, opts.shiftKhz * 1000.0);

                std::cerr << "airspyhf_decimator: locked input rate="
                          << effectiveInputRate
                          << " outputRate=" << effectiveOutputRate << " source="
                          << ((opts.inputRate > 0.0) ? "--input-rate"
                                                     : "first_packet_header")
                          << "\n";
            }

            const double rateErrorPpm =
                1e6 *
                std::abs(static_cast<double>(packet.sampleRate) -
                         effectiveInputRate) /
                effectiveInputRate;
            if (rateErrorPpm > opts.rateTolerancePpm) {
                if (opts.strictInputRate) {
                    throw std::runtime_error(
                        "Strict input-rate mismatch in packet header");
                }
                ++sampleRateFieldWarnings;
                if (sampleRateFieldWarnings <= 10 ||
                    (sampleRateFieldWarnings % 100) == 0) {
                    std::cerr
                        << "airspyhf_decimator: bad incoming sample_rate field="
                        << packet.sampleRate
                        << " expected=" << effectiveInputRate
                        << " error_ppm=" << rateErrorPpm
                        << " warnings=" << sampleRateFieldWarnings << "\n";
                }
            }

            auto stageInput = std::move(packet.samples);
            inputSamplesProcessed += stageInput.size();

            if (!frequencyShifter || !timestampEncoder) {
                std::cerr << "airspyhf_decimator: internal initialization "
                             "incomplete, skipping packet sequence="
                          << packet.sequence << "\n";
                continue;
            }

            auto processStart = std::chrono::steady_clock::now();
            frequencyShifter->mix(stageInput);
            auto afterStage1 = stage1.process(stageInput);
            auto afterStage2 = stage2.process(afterStage1);
            auto decimated = stage3.process(afterStage2);
            processingTime += (std::chrono::steady_clock::now() - processStart);
            outputSamplesProduced += decimated.size();

            if (!decimated.empty()) {
                buffer.insert(buffer.end(), decimated.begin(), decimated.end());
            }

            while (buffer.size() >= payloadSamples) {
                std::vector<std::complex<float>> frame;
                frame.reserve(opts.packetSamples);
                frame.push_back(timestampEncoder->headerForSample(samplesSent));
                frame.insert(frame.end(), buffer.begin(),
                             buffer.begin() + payloadSamples);
                streamer.send(frame);
                ++framesSent;
                buffer.erase(buffer.begin(), buffer.begin() + payloadSamples);
                samplesSent += payloadSamples;
            }

            auto now = std::chrono::steady_clock::now();
            if (now - lastPerfLog >= std::chrono::seconds(1)) {
                const double elapsedSec =
                    std::chrono::duration<double>(now - runStart).count();
                const double processingSec =
                    std::chrono::duration<double>(processingTime).count();
                const double inputRate =
                    (elapsedSec > 0.0)
                        ? (static_cast<double>(inputSamplesProcessed) /
                           elapsedSec)
                        : 0.0;
                const double outputRateMeasured =
                    (elapsedSec > 0.0)
                        ? (static_cast<double>(outputSamplesProduced) /
                           elapsedSec)
                        : 0.0;
                const double zmqBytesPerSec =
                    (elapsedSec > 0.0)
                        ? (static_cast<double>(zmqBytesRead) / elapsedSec)
                        : 0.0;
                const double zmqComplexPerSec =
                    (elapsedSec > 0.0)
                        ? (static_cast<double>(inputSamplesProcessed) /
                           elapsedSec)
                        : 0.0;
                const double frameRate =
                    (elapsedSec > 0.0)
                        ? (static_cast<double>(framesSent) / elapsedSec)
                        : 0.0;
                const double processingDuty =
                    (elapsedSec > 0.0) ? (100.0 * processingSec / elapsedSec)
                                       : 0.0;
                const double measuredRateErrorPpm =
                    (effectiveInputRate > 0.0)
                        ? (1e6 *
                           std::abs(zmqComplexPerSec - effectiveInputRate) /
                           effectiveInputRate)
                        : 0.0;

                if (measuredRateErrorPpm > opts.rateTolerancePpm) {
                    if (opts.strictInputRate) {
                        throw std::runtime_error("Strict input-rate mismatch "
                                                 "in measured stream rate");
                    }
                    ++measuredRateWarnings;
                    if (measuredRateWarnings <= 10 ||
                        (measuredRateWarnings % 100) == 0) {
                        std::cerr << "airspyhf_decimator: bad incoming "
                                     "measured sample rate="
                                  << zmqComplexPerSec
                                  << " expected=" << effectiveInputRate
                                  << " error_ppm=" << measuredRateErrorPpm
                                  << " warnings=" << measuredRateWarnings
                                  << "\n";
                    }
                }

                std::cerr << "airspyhf_decimator: perf zmq_Bps="
                          << zmqBytesPerSec
                          << " zmq_complex_sps=" << zmqComplexPerSec
                          << " in_sps=" << inputRate
                          << " out_sps=" << outputRateMeasured
                          << " frames_per_s=" << frameRate
                          << " cpu_duty_pct=" << processingDuty
                          << " buffer_samples=" << buffer.size()
                          << " zmq_packets=" << zmqPacketsReceived
                          << " malformed=" << receiver.malformedPackets()
                          << " dropped=" << droppedPackets
                          << " out_of_order=" << outOfOrderPackets << "\n";

                if (haveSequence && lastZmqTimestampUs > firstZmqTimestampUs) {
                    const double streamDurationSec =
                        static_cast<double>(lastZmqTimestampUs -
                                            firstZmqTimestampUs) /
                        1'000'000.0;
                    const double timestampRate =
                        (streamDurationSec > 0.0)
                            ? (static_cast<double>(inputSamplesProcessed) /
                               streamDurationSec)
                            : 0.0;
                    std::cerr << "airspyhf_decimator: zmq_timestamp_rate_sps="
                              << timestampRate
                              << " first_ts_us=" << firstZmqTimestampUs
                              << " last_ts_us=" << lastZmqTimestampUs << "\n";
                }
                lastPerfLog = now;
            }
        }

        std::cerr << "airspyhf_decimator: stopping packets="
                  << zmqPacketsReceived
                  << " malformed=" << receiver.malformedPackets()
                  << " dropped=" << droppedPackets
                  << " out_of_order=" << outOfOrderPackets
                  << " sample_rate_field_warnings=" << sampleRateFieldWarnings
                  << " measured_rate_warnings=" << measuredRateWarnings << "\n";
    } catch (const ArgsError &err) {
        std::cerr << "Argument error: " << err.what() << "\n";
        printUsage(argv[0]);
        return 64;
    } catch (const std::exception &err) {
        std::cerr << "Fatal error: " << err.what() << "\n";
        return 1;
    }

    return 0;
}
