#include "../common_inc.hpp"
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <cstdlib>
#include <cstdio>
#include <charconv>


static inline void write_u32(uint32_t v) {
    char buf[16];
    auto res = std::to_chars(buf, buf + sizeof(buf) - 1, v);
    *res.ptr = '\0';
    usb::writeLine(buf);
}


static constexpr size_t kLineCap   = 512;
static constexpr size_t kMaxFlags  = 12;
static constexpr size_t kMaxKeyLen = 96;

struct Flag { std::string_view key; int value; float fvalue; };

struct FlagsView {
    const Flag* data{};
    size_t      size{};

    bool get(std::string_view k, int& out) const {
        for (size_t i = 0; i < size; ++i) if (data[i].key == k) { out = data[i].value; return true; }
        return false;
    }
    bool get(std::string_view k, float& out) const {
        for (size_t i = 0; i < size; ++i) if (data[i].key == k) { out = data[i].fvalue; return true; }
        return false;
    }
};

static inline bool is_space(unsigned char c){ return std::isspace(c) != 0; }

static inline bool parse_flag_token(char* tok, std::string_view& key, int& iv, float& fv) {
    char* p = tok;
    while (*p == '-') ++p;
    if (*p == '\0') return false;
    char* eq = std::strchr(p, '=');
    if (eq) {
        *eq = '\0';
        key = std::string_view(p);
        const char* s = eq + 1;

        char* endi = nullptr; long vi = std::strtol(s, &endi, 10);
        char* endf = nullptr; float vf = std::strtof(s, &endf);

        if (endf != s) { fv = vf; } else { fv = static_cast<float>(vi); }
        if (endi != s) { iv = static_cast<int>(vi); } else { iv = 1; }
    } else {
        key = std::string_view(p);
        iv = 1;
        fv = 1.0f;
    }
    return true;
}

static inline void handleline(
    char* line,
    char* keybuf, size_t keybuf_cap, size_t& keylen_out,
    Flag* flags, size_t flags_cap, size_t& flag_count_out)
{
    keylen_out = 0;
    flag_count_out = 0;
    if (!line || !*line) return;
    char* s = line;
    bool first_cmd_tok = true;
    while (*s) {
        while (*s && is_space(static_cast<unsigned char>(*s))) ++s;
        if (!*s) break;
        char* tok = s;
        while (*s && !is_space(static_cast<unsigned char>(*s))) ++s;
        if (*s) { *s = '\0'; ++s; }
        if (tok[0] == '-') {
            if (flags_cap == 0) continue;
            std::string_view k; int vi; float vf;
            if (parse_flag_token(tok, k, vi, vf)) {
                bool replaced = false;
                for (size_t i = 0; i < flag_count_out; ++i) {
                    if (flags[i].key == k) { flags[i].value = vi; flags[i].fvalue = vf; replaced = true; break; }
                }
                if (!replaced && flag_count_out < flags_cap) {
                    flags[flag_count_out++] = {k, vi, vf};
                }
            }
        } else {
            size_t toklen = std::strlen(tok);
            if (!first_cmd_tok) {
                if (keylen_out + 1 < keybuf_cap) keybuf[keylen_out++] = ' ';
            }
            for (size_t i = 0; i < toklen && keylen_out + 1 < keybuf_cap; ++i) {
                unsigned char c = static_cast<unsigned char>(tok[i]);
                if (c >= 'A' && c <= 'Z') c = c + 32;
                keybuf[keylen_out++] = static_cast<char>(c);
            }
            first_cmd_tok = false;
        }
    }
    if (keybuf_cap) keybuf[(keylen_out < keybuf_cap) ? keylen_out : (keybuf_cap - 1)] = '\0';
}



// CLI Welcome
static inline void cmd_welcome()         { usb::println("Welcome"); }
static inline void cmd_help()            { usb::println("Help"); }

// TEMP
static inline void cmd_sys_temp(FlagsView flags) {
    int d;
    if (flags.get("raw", d) || flags.get("r", d)) { usb::println(static_cast<uint32_t>(temp::raw())); return; }
    bool k = flags.get("k", d) || flags.get("kelvin", d);
    bool f = flags.get("f", d) || flags.get("fahrenheit", d);
    const char* u = k ? " K" : (f ? " F" : " C");
    float v = k ? temp::kelvin() : (f ? temp::fahrenheit() : temp::celcius());
    usb::println("TEMP: ", v, u);
}

// POWER
static inline void cmd_sys_pwr(FlagsView flags) {
    int d;
    if (flags.get("raw", d) || flags.get("r", d)) { usb::println(static_cast<uint32_t>(pwr::raw())); return; }
    usb::println("V_bus: ", pwr::volts(), " V");
}

// LED Status
static inline void cmd_sys_led(FlagsView flags) {
    int v, d;
    if (flags.get("on", d))    { led::stopToggle(); led::setLed(true);  return; }
    if (flags.get("off", d))   { led::stopToggle(); led::setLed(false); return; }
    if (flags.get("stop", d))  { led::stopToggle(); led::setLed(false); return; }
    if (flags.get("set", v))   { led::stopToggle(); led::setLed(v != 0); return; }
    if (flags.get("blink", v)) { led::startToggle(static_cast<float>(v)); return; }
}

// Gate Driver Status
static inline void cmd_view_drv(FlagsView flags) {
    Driver8323s::Status s{};
    if (!drv.readStatus(s)) { usb::println("DRV: ERR_READ_FAIL"); return; }
    int d;
    if (flags.get("all", d) || flags.get("a", d)) {
        auto b11 = [](uint16_t x) {
            static char b[12];
            for (int i = 0; i < 11; ++i) b[i] = ((x >> (10 - i)) & 1u) ? '1' : '0';
            b[11] = '\0'; return b;
        };
        usb::println("S1:", b11(s.raw_status1), " S2:", b11(s.raw_status2), " C2:", b11(s.raw_ctrl2));
        usb::println("C3:", b11(s.raw_ctrl3), " C4:", b11(s.raw_ctrl4), " C5:", b11(s.raw_ctrl5), " C6:", b11(s.raw_ctrl6));
        return;
    }
    usb::println("UVLO:", (int)s.uvlo, " CPUV:", (int)s.cp_uv, " OCP:", (int)(s.vds_ocp || s.sa_oc || s.sb_oc || s.sc_oc), " GDF:", (int)s.gdf);
}

// Encoder Status
static inline void cmd_view_enc(FlagsView flags) {
    int d;
    const auto& a = encoder.angleData();
    if (flags.get("diag", d) || flags.get("d", d)) {
        uint16_t reg = encoder.readDiaagc();
        if (reg == 0xFFFFu) { usb::println("ENC: ERR_FAIL"); return; }
        reg = align14(reg) & 0x0FFFu;
        char b[13]; for (int i = 0; i < 12; ++i) b[i] = (reg & (1u << (11 - i))) ? '1' : '0'; b[12] = '\0';
        usb::println("DIAG:", b); return;
    }
    if (flags.get("raw", d) || flags.get("r", d)) { usb::println("RAW:", (int)a.rawAngle); return; }
    if (flags.get("deg", d) || flags.get("d", d)) { usb::println("DEG:", a.degrees); return; }
    if (flags.get("rad", d)) { usb::println("RAD:", a.radians); return; }
    if (flags.get("elec", d) || flags.get("e", d)) { usb::println("ELEC:", foc::status.currTheta); return; }
    usb::println("DEG:", a.degrees, " ELEC:", foc::status.currTheta);
}

// FOC Status
static inline void cmd_view_foc(FlagsView flags) {
    int d;
    if (flags.get("adc", d) || flags.get("a", d)) { usb::println("IA:", foc::status.cA, " IB:", foc::status.cB, " IC:", foc::status.cC); return; }
    if (flags.get("dq", d)  || flags.get("i", d)) { usb::println("IQ:", foc::status.Iq, " ID:", foc::status.Id); return; }
    if (flags.get("out", d) || flags.get("o", d)) { usb::println("VA:", foc::status.phaseA, " VB:", foc::status.phaseB, " VC:", foc::status.phaseC); return; }
    usb::println("IQ:", foc::status.Iq, " ID:", foc::status.Id, " MECH:", controller.status().curr_mechAng, " ELEC: ", controller.status().elecAng);
}

// Controller Status
static inline void cmd_ctrl_state(FlagsView flags) {
    int d;
    auto& s = controller.status();
    bool f = (flags.get("v", d) || flags.get("vel", d) || flags.get("velocity", d));
    if (f) usb::println("VEL:", s.curr_vel, " TARGET:", s.TarVel, " REV:", s.revolution_count);

    if (flags.get("p", d) || flags.get("pos", d) || flags.get("position", d)) {
        usb::println("POS:", s.curr_pos, " TARGET:", s.TarPos, " REV:", s.revolution_count); f = true;
    }
    if (flags.get("t", d) || flags.get("trq", d) || flags.get("torque", d)) {
        usb::println("TarId:", foc::status.TarId, " TarIq:", foc::status.TarIq, " Vd:", foc::status.Vd, " Vq:", foc::status.Vq); f = true;
    }

    if (!f) {
        const char* m[] = {"IDLE", "TRQ", "VEL", "POS", "CAL"};
        const char* tm = (controller.config().torque_mode == Controller::TorqueMode::Voltage) ? "VOLT" : "CURR";
        usb::println("MODE:", m[int(s.mode)], " TYPE:", tm, " ARMED:", (int)s.armed, " CALIB:", (int)s.isCalibrated);
    }
}

// Controller Control
static inline void cmd_ctrl_set(FlagsView flags) {
    float v, vd = 0.0f, vq = 0.0f;

    if (flags.get("v", v) || flags.get("vel", v) || flags.get("velocity", v)) { controller.set_velocity(v); return; }
    if (flags.get("p", v) || flags.get("pos", v) || flags.get("position", v)) { controller.set_position(v); return; }
    if (flags.get("t", v) || flags.get("trq", v) || flags.get("torque", v))   { controller.set_torque(0, v / 1000.0f); } //turn input from mA to A

    if (flags.get("Vd", vd) || flags.get("Vq", vq)) {
        flags.get("Vd", vd); flags.get("Vq", vq);
        controller.set_torque(vd, vq);
        return;
    }
}

// Set Zero point
static inline void cmd_ctrl_setzero() {
	controller.set_zero();
    usb::println("ZEROED");
}

// IDLE motor
static inline void cmd_ctrl_idle() {
    controller.status().mode = Controller::Mode::Idle;
    usb::println("IDLE");
}

// Arm motor
static inline void cmd_ctrl_arm() {
	controller.arm();
	usb::println("ARM");
}

// Disarm motor
static inline void cmd_ctrl_disarm() {
	controller.disarm();
	usb::println("DISARM");
}

// Calibrate Cycle
static inline void cmd_ctrl_cal() {
    controller.calibrate();
}

// Get Config Parameters
static inline void cmd_conf_get(FlagsView flags) {
    auto& c = controller.config();
    int d;

    if (flags.get("pid", d)) 	   { usb::println("I_PI=", c.i_kp, "/", c.i_ki, ", V_PI=", c.v_kp, "/", c.v_ki, ", P_PID=", c.p_kp, "/", c.p_ki, "/", c.p_kd); return; }
    if (flags.get("lim", d)) 	   { usb::println("VMAX=", c.max_voltage, ", IMAX=", c.max_current, ", VELMAX=", c.max_vel); return; }
    if (flags.get("hw", d))        { usb::println("PP=", c.pole_pairs, ", RSH=", c.shunt_res, ", ADCG=", c.adc_gain, ", SGN=", (int)c.elec_s); return; }
    if (flags.get("freq", d)) 	   { usb::println("CFREQ=", c.current_loop_freq, ", VFREQ=", c.vel_loop_freq); return; }

    if (flags.get("pp", d))        { usb::println("PP=", c.pole_pairs); return; }
    if (flags.get("sgn", d))       { usb::println("SGN=", (int)c.elec_s); return; }
    if (flags.get("eoff", d))      { usb::println("EOFF=", c.elec_offset); return; }
    if (flags.get("rsh", d))       { usb::println("RSH=", c.shunt_res); return; }
    if (flags.get("adcg", d))      { usb::println("ADCG=", c.adc_gain); return; }
    if (flags.get("valpha", d))    { usb::println("VEL_ALPHA=", c.vel_alpha); return; }
    if (flags.get("palpha", d))    { usb::println("POS_ALPHA=", c.pos_alpha); return; }

    if (flags.get("all", d)) {
        usb::println("HW: PP=", c.pole_pairs, ", SGN=", (int)c.elec_s, ", RSH=", c.shunt_res, ", ADCG=", c.adc_gain);
        usb::println("LOOP: C=", c.current_loop_freq, ", V=", c.vel_loop_freq, ", EOFF=", c.elec_offset);
        usb::println("PID: I=", c.i_kp, "/", c.i_ki, ", V=", c.v_kp, "/", c.v_ki, ", P=", c.p_kp, "/", c.p_ki, "/", c.p_kd);
        usb::println("LIM: V=", c.max_voltage, ", I=", c.max_current, ", VEL=", c.max_vel);
        usb::println("FILT: V_A=", c.vel_alpha, ", P_A=", c.pos_alpha);
    }
}

static inline void cmd_conf_set(FlagsView flags) {
    auto& c = controller.config();
    float fv;
    int iv;

    if (flags.get("ikp", fv)) { c.i_kp = fv; }
    if (flags.get("iki", fv)) { c.i_ki = fv; }
    if (flags.get("vkp", fv)) { c.v_kp = fv; }
    if (flags.get("vki", fv)) { c.v_ki = fv; }
    if (flags.get("pkp", fv)) { c.p_kp = fv; }
    if (flags.get("pki", fv)) { c.p_ki = fv; }
    if (flags.get("pkd", fv)) { c.p_kd = fv; }

    if (flags.get("valpha", fv)) { c.vel_alpha = fv; }
    if (flags.get("palpha", fv)) { c.pos_alpha = fv; }

    if (flags.get("vmax", fv))   { c.max_voltage = fv; }
    if (flags.get("imax", fv))   { c.max_current = fv; }
    if (flags.get("velmax", fv)) { c.max_vel = fv; }

    if (flags.get("rsh", fv))    { c.shunt_res = fv; }
    if (flags.get("adcg", fv))   { c.adc_gain = fv; }
    if (flags.get("pp", iv))     { c.pole_pairs = static_cast<uint16_t>(iv); }
    if (flags.get("sgn", iv))    { c.elec_s = static_cast<int8_t>(iv); }
    if (flags.get("eoff", iv))   { c.elec_offset = static_cast<uint16_t>(iv & 0x3FFF); }

    if (flags.get("cfreq", iv)) {
        c.current_loop_freq = static_cast<uint32_t>(iv);
        controller.set_current_loop_freq(c.current_loop_freq);
    }
    if (flags.get("vfreq", iv)) {
        c.vel_loop_freq = static_cast<uint32_t>(iv);
        controller.set_velocity_loop_freq(c.vel_loop_freq);
    }

    if (controller.config_write()) {
        usb::println("CONF=WRITE_OK");
    } else {
        usb::println("CONF=WRITE_FAIL");
    }
}


static inline void commands(std::string_view key, FlagsView flags)
{
    if (key.empty()) return;

    if (key == "cdrv")                    { (void)cmd_welcome(); return; }    // print a welcome banner with basic info
    if (key == "cdrv help")               { (void)cmd_help(); return; }    // print a command table

    if (key == "cdrv sys temp") { cmd_sys_temp(flags); return; } 	// print temperature data
    if (key == "cdrv sys pwr")  { cmd_sys_pwr(flags);  return; } 	// print power data
    if (key == "cdrv sys led")  { cmd_sys_led(flags);  return; } 	// control status LED

    if (key == "cdrv view drv") { cmd_view_drv(flags); return; }	// print info about the gate driver
    if (key == "cdrv view enc") { cmd_view_enc(flags); return; }	// print info about the encoder
    if (key == "cdrv view foc") { cmd_view_foc(flags); return; }	// print info about the FOC status

    if (key == "cdrv ctrl state")   { cmd_ctrl_state(flags); return; }	// print info about the controller status
    if (key == "cdrv ctrl set")     { cmd_ctrl_set(flags);   return; }	// set / control the motor
    if (key == "cdrv ctrl idle")    { cmd_ctrl_idle();       return; }	// send motor into idle state
    if (key == "cdrv ctrl arm")     { cmd_ctrl_arm();        return; }  // Arm motor
    if (key == "cdrv ctrl disarm")  { cmd_ctrl_disarm();     return; }  // Disarm motor
    if (key == "cdrv ctrl zero") { cmd_ctrl_setzero();    return; }  // Set current position as zero
    if (key == "cdrv ctrl cal")     { cmd_ctrl_cal();        return; }	// start calibration process

    if (key == "cdrv conf get") { cmd_conf_get(flags); return; }	// Retrieve Config Parameters
    if (key == "cdrv conf set") { cmd_conf_set(flags); return; }	// Set Config Parameters
}

void cli_poll()
{
    static char line[kLineCap];
    if (usb::readLine(line, sizeof(line))) {
        char keybuf[kMaxKeyLen]{};
        size_t keylen = 0;
        Flag flagbuf[kMaxFlags];
        size_t nflags = 0;

        handleline(line, keybuf, sizeof(keybuf), keylen,
                        flagbuf, sizeof(flagbuf)/sizeof(flagbuf[0]), nflags);
        std::string_view key{keybuf, keylen};
        FlagsView fv{flagbuf, nflags};
        commands(key, fv);
    }
}
