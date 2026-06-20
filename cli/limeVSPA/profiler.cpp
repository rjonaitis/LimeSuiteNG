#include "chips/LA9310/vspa/l1-trace.h"

#include <sstream>

extern std::string GetMsgName(uint32_t msg);

static const double clockRate = 30.72e6 * 4;
static const double tickDuration = 1 / clockRate;

enum class ePhase {
    Begin,
    End,
    AsyncBegin,
    AsyncEnd,
    Instant,
    Counter,
};

struct Event {
    std::string name;
    std::string category;
    ePhase phase;
    uint64_t timestamp;
    uint32_t pid;
    uint32_t tid;
    uint32_t id;
};

static std::string ToString(ePhase phase)
{
    switch(phase)
    {
    case ePhase::Begin: return "B";
    case ePhase::End: return "E";
    case ePhase::AsyncBegin: return "B";
    case ePhase::AsyncEnd: return "E";
    case ePhase::Instant: return "I";
    case ePhase::Counter: return "C";
    default: return "";
    }
}

static std::string ToString(Event& evt)
{
    std::stringstream ss;
    ss << "{ " << "\"cat\": \"" << evt.category << "\"" << ",\"ts\":"
       << uint64_t(double(evt.timestamp * tickDuration) * 1e6) << ",\"pid\":" << evt.pid << ",\"tid\":" << evt.tid << ",\"ph\":" << "\""
       << ToString(evt.phase) << "\"" << ",\"name\": \"" << evt.name << "\"";
    if (evt.id)
        ss << ",\"id\":" << evt.id;
    ss << "}";
    return ss.str();
}

static std::string NameWithTag(const std::string& name, uint32_t tag)
{
    char ctemp[64];
    sprintf(ctemp, "%s:%X", name.c_str(), tag);
    return ctemp;
}

static std::string OpName(uint32_t op)
{
    switch(op)
    {
        case T_XFER_BUFFER: return "DMA";
        case T_QEC_TX_BUFFER: return "TX_QEC";
        case T_QEC_RX_BUFFER: return "RX_QEC";
        case T_DEC_BUFFER: return "DEC";
        case T_INT_BUFFER: return "INT";
        case T_UNDERRUN: return "UDR";
        case T_OVERRUN: return "OVR";
        default:
        {
            char ctemp[32];
            sprintf(ctemp, "%X", op);
            return ctemp;
        }
    }
}

Event Convert(const l1_trace_data_t& data)
{
    Event evt;
    evt.pid = (data.msg >> 28) & 0xf;
    evt.tid = (data.msg >> 24) & 0xf;
    evt.id = data.param;
    evt.phase = static_cast<ePhase>((data.msg >> 21) & 0x7);
    evt.timestamp = data.cnt;
    evt.name = NameWithTag(OpName(data.msg & 0x1FFFFF), data.param);
    if (evt.pid == 2)
    {
        evt.pid = evt.tid + 1000;
        evt.tid = data.param;
        evt.name = OpName(data.msg & 0x1FFFFF);
    }
    // evt.name = OpName(data.msg & 0x1FFFFF);
    evt.category = "";
    return evt;
}

void ToTraceFile(std::ofstream& ofs, const std::vector<l1_trace_data_t> events)
{
    for (const auto& e : events)
    {
        if (e.msg == 0)
            break;

        Event evt = Convert(e);
        // if (!evt.pid || evt.name.empty())
        //     continue;

        // if (!baseTime)
        //     baseTime = data[i].cnt;

        // evt.timestamp -= baseTime;
        ofs << ToString(evt) << '\n';//std::endl;
    }
}