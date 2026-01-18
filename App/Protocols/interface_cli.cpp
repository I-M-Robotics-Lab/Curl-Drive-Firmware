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
static inline void cmd_temp_raw()        { usb::println(static_cast<uint32_t>(temp::raw())); }
static inline void cmd_temp_kelvin()     { usb::println("TEMP: ", temp::kelvin(),     " K"); }
static inline void cmd_temp_celcius()    { usb::println("TEMP: ", temp::celcius(),    " C"); }
static inline void cmd_temp_fahrenheit() { usb::println("TEMP: ", temp::fahrenheit(), " F"); }

// VBAT
static inline void cmd_power_raw()       { usb::println(static_cast<uint32_t>(vbat::raw())); }
static inline void cmd_power_voltage()   { usb::println("VBAT: ", vbat::volts(), " V"); }

// LED
static inline void cmd_led(FlagsView flags) {
    int v;
    if (flags.get("off", v)) { led::stopToggle(); led::setLed(false); return; }
    if (flags.get("on",  v)) { led::stopToggle(); led::setLed(true);  return; }
}

static inline void cmd_led_toggle(FlagsView flags) {
    int v;
    if (flags.get("off", v)) { led::stopToggle(); return; }
    if (flags.get("start", v)) {
        int hzInt;
        if (flags.get("freq", hzInt)) { led::startToggle(static_cast<float>(hzInt)); }
        else                           { led::startToggle(); }
        return;
    }
}

// driver
static inline void cmd_drv_status(FlagsView flags)
{
    Driver8323s::Status s{};
    if (!drv.readStatus(s)) { usb::println("[DRV8323S] readStatus failed"); return; }

    int v = 0;
    if (flags.get("all", v)) {
        auto fmt11 = [](uint16_t x, char* out) {
            for (int b = 10, i = 0; b >= 0; --b, ++i) out[i] = ((x >> b) & 1u) ? '1' : '0';
            out[11] = '\0';
        };

        char s1[12], s2[12], c2[12], c3[12], c4[12], c5[12], c6[12];
        fmt11(s.raw_status1, s1);
        fmt11(s.raw_status2, s2);
        fmt11(s.raw_ctrl2,   c2);
        fmt11(s.raw_ctrl3,   c3);
        fmt11(s.raw_ctrl4,   c4);
        fmt11(s.raw_ctrl5,   c5);
        fmt11(s.raw_ctrl6,   c6);

        usb::println("[DRV8323S] S1=", s1, "  S2=", s2, "  C2=", c2);
        usb::println("  C3=", c3, "  C4=", c4, "  C5=", c5);
        usb::println("  C6=", c6);
        return;
    }

    usb::println(" UVLO=", (int)s.uvlo, " CPUV=", (int)s.cp_uv, " OCP=", (int)(s.vds_ocp || s.sa_oc || s.sb_oc || s.sc_oc), " GDF=", (int)s.gdf);
}


// Encoder
static inline void cmd_enc_diag() {
    uint16_t v=encoder.readDiaagc(); if(v==0xFFFFu){ usb::println("[AS5047] DIAAGC=READ_FAIL"); return; }
    v=align14(v)&0x0FFFu; char bits[13]; for(int i=11,k=0;i>=0;--i,++k) bits[k]=(v&(1u<<i))?'1':'0'; bits[12]='\0';
    usb::println("[AS5047] DIAAGC=",bits);
}
static inline void cmd_enc_angle(FlagsView flags) {
	int v = 0;
    if (flags.get("raw", v)) { const auto& a = encoder.angleData(); usb::println("[AS5047] rawAngle=", (int)a.rawAngle); }
    if (flags.get("rec", v)) { const auto& a = encoder.angleData(); usb::println("[AS5047] rectifiedAngle=", (int)a.rectifiedAngle); }
    if (flags.get("rad", v)) { const auto& a = encoder.angleData(); usb::println("[AS5047] rad=", (int)a.radians); }
    if (flags.get("deg", v)) { const auto& a = encoder.angleData(); usb::println("[AS5047] deg=", (int)a.degrees); }
    if (flags.get("elec", v)) { usb::println("[AS5047] electricalAngle=", (int)foc::status.currTheta); }
    if (flags.get("t", v)) { usb::println("[AS5047] t=", (int)foc::status.currT); }
    return;
}

// FOC & controller
static inline void cmd_enc_is_armed() { usb::println("armed=", controller.isArmed()); }
static inline void cmd_con_phases() { usb::println("Phase A=", foc::status.phaseA, "  Phase B=", foc::status.phaseB, "  Phase C=", foc::status.phaseC); return; }
static inline void cmd_con_curr() { usb::println("Iq=", foc::status.Iq, "  Id=", foc::status.Id); return; }
static inline void cmd_con_angle() { usb::println("Electrical Angle=", foc::status.currTheta, "Mechanical Angle=", controller.status().curr_mechAng); return; }
static inline void cmd_con_adcoffset() { usb::println("offset_A=", controller.config().offset_a, "  offset_B=", controller.config().offset_b, "  offset_C=", controller.config().offset_c); return; }
static inline void cmd_con_adc() { usb::println("cA=", foc::status.cA, "  cB=", foc::status.cB, "  cC=", foc::status.cC); return; }
static inline void cmd_con_velocity() { usb::println("velocity_raw=", controller.status().curr_vel_raw, " rad/s, velocity=", controller.status().curr_vel, " rad/s"); }
static inline void cmd_con_tmode() {
	const char* tm = (controller.status().torque_mode == Controller::TorqueMode::Voltage) ? "Voltage" : "Current";
    usb::println("Torque Mode=", tm);
}
static inline void cmd_con_mode() {
    const char* m;
    switch (controller.status().mode) {
        case Controller::Mode::Idle:        m = "Idle"; break;
        case Controller::Mode::Torque:      m = "Torque"; break;
        case Controller::Mode::Velocity:    m = "Velocity"; break;
        case Controller::Mode::Position:    m = "Position"; break;
        case Controller::Mode::Calibrating: m = "Calibrating"; break;
        default:                            m = "Unknown"; break;
    }
    usb::println("Mode=", m);
}

static inline void cmd_con_config(FlagsView flags) {
    auto& cfg = controller.config();
    int v = 0;
    if (flags.get("pid", v)) usb::println("pid:", " i_kp:", cfg.i_kp, ", i_ki:", cfg.i_ki, ", v_kp:", cfg.v_kp, ", v_ki:", cfg.v_ki, ", p_kp:", cfg.p_kp, ", p_ki:", cfg.p_ki, " p_kd:", cfg.p_kd);
    if (flags.get("freq", v)) usb::println("freq:", cfg.current_loop_freq, " Hz");
    if (flags.get("pp", v))   usb::println("pp:", cfg.pole_pairs);
    if (flags.get("sgn", v))  usb::println("sgn:", int(cfg.elec_s));
    if (flags.get("eoff", v)) usb::println("eoff:", cfg.elec_offset);
    if (flags.get("rsh", v))  usb::println("rsh:", cfg.shunt_res, " ohm");
    if (flags.get("adcg", v)) usb::println("adcg:", cfg.adc_gain);
    if (flags.get("vmax", v)) usb::println("vmax:", cfg.max_voltage, " V");
    if (flags.get("imax", v)) usb::println("imax:", cfg.max_current, " A");
    if (flags.get("velmax", v)) usb::println("velmax: ", cfg.max_vel, " RPM");
    if (flags.get("vel_alpha", v)) usb::println("velocity_alpha: ", cfg.vel_alpha);
    if (flags.get("all", v)) usb::println("freq:", cfg.current_loop_freq, " Hz, pp:", cfg.pole_pairs, ", sgn:", int(cfg.elec_s), ", eoff:", cfg.elec_offset, ", rsh:", cfg.shunt_res, " ohm, adcg:", cfg.adc_gain, ", vmax:", cfg.max_voltage, " V, imax:", cfg.max_current, " A");
}

static inline void cmd_con_config_write(FlagsView flags) {
	auto& cfg = controller.config();
    int cfreq_i, vfreq_i, pp_i, eoff_i, sgn_i;
    float rsh_f, adcg_f, vmax_f, imax_f, velmax_f, ikp_f, iki_f, vkp_f, vki_f, pkp_f, pki_f, pkd_f, vel_alpha;

    if (flags.get("cfreq", cfreq_i)) { cfg.current_loop_freq = static_cast<uint32_t>(cfreq_i); controller.set_current_loop_freq(cfg.current_loop_freq); }
    if (flags.get("vfreq", vfreq_i)) { cfg.vel_loop_freq = static_cast<uint32_t>(vfreq_i); controller.set_velocity_loop_freq(cfg.vel_loop_freq); }
    if (flags.get("pp",   pp_i))   { cfg.pole_pairs  = static_cast<uint16_t>(pp_i); }
    if (flags.get("sgn",  sgn_i))  { cfg.elec_s      = static_cast<int8_t>(sgn_i); }
    if (flags.get("eoff", eoff_i)) { cfg.elec_offset = static_cast<uint16_t>(eoff_i & 0x3FFF); }
    if (flags.get("rsh",  rsh_f))  cfg.shunt_res   = rsh_f;
    if (flags.get("adcg", adcg_f)) cfg.adc_gain    = adcg_f;
    if (flags.get("vmax", vmax_f)) cfg.max_voltage = vmax_f;
    if (flags.get("imax", imax_f)) cfg.max_current = imax_f;
    if (flags.get("velmax", velmax_f)) cfg.max_vel = velmax_f;
    if (flags.get("ikp",  ikp_f))  cfg.i_kp = ikp_f;
    if (flags.get("iki",  iki_f))  cfg.i_ki = iki_f;
    if (flags.get("vkp",  vkp_f))  cfg.v_kp = vkp_f;
    if (flags.get("vki",  vki_f))  cfg.v_ki = vki_f;
    if (flags.get("pkp",  pkp_f))  cfg.p_kp = pkp_f;
    if (flags.get("pki",  pki_f))  cfg.p_ki = pki_f;
    if (flags.get("pkd",  pkd_f))  cfg.p_kd = pkd_f;
    if (flags.get("vel_alpha", vel_alpha)) cfg.vel_alpha = vel_alpha;

    (void)controller.config_write();
}

static inline void cmd_con_calibrate() { controller.calibrate(); }
static inline void cmd_con_iscalibrated() { usb::println(controller.status().isCalibrated); }

// Control Modes

static inline void cmd_con_idle() { controller.status().mode = Controller::Mode::Idle; }
static inline void cmd_con_torque(FlagsView flags) {
    float d = 0.0f, q = 0.0f;
    flags.get("Vd", d);
    flags.get("Vq", q);
    flags.get("Id", d);
    flags.get("Iq", q);
    controller.set_torque(d, q);
}
static inline void cmd_con_velocity(FlagsView flags) {
    float vel = 0.0f;
    if (flags.get("vel", vel)) {
        controller.set_velocity(vel);
    }
}
static inline void cmd_con_position(FlagsView flags) {
    float pos = 0.0f;
    if (flags.get("pos", pos)) {
        controller.set_position(pos);
    }
}

static inline void commands(std::string_view key, FlagsView flags)
{
    if (key.empty()) return;
    if (key == "curl-drive")                    { (void)cmd_welcome();         return; }
    if (key == "curl-drive help")               { (void)cmd_help();            return; }

    if (key == "curl-drive temp raw")           { (void)cmd_temp_raw();        return; }
    if (key == "curl-drive temp kelvin")        { (void)cmd_temp_kelvin();     return; }
    if (key == "curl-drive temp celcius")       { (void)cmd_temp_celcius();    return; }
    if (key == "curl-drive temp fahrenheit")    { (void)cmd_temp_fahrenheit(); return; }

    if (key == "curl-drive power raw")          { (void)cmd_power_raw();       return; }
    if (key == "curl-drive power voltage")      { (void)cmd_power_voltage();   return; }

    if (key == "curl-drive led")                { (void)cmd_led(flags);        return; }
    if (key == "curl-drive led toggle")         { (void)cmd_led_toggle(flags); return; }

    if (key == "curl-drive driver status")      { (void)cmd_drv_status(flags);      return; }

    if (key == "curl-drive encoder diag")  		{ (void)cmd_enc_diag();    return; }
    if (key == "curl-drive encoder angle")      { (void)cmd_enc_angle(flags);     return; }

    if (key == "curl-drive controller phases")  { (void)cmd_con_phases(); return; }
    if (key == "curl-drive controller current") { (void)cmd_con_curr(); return; }
    if (key == "curl-drive controller angle")   { (void)cmd_con_angle(); return; }
    if (key == "curl-drive controller adcoffset") { (void)cmd_con_adcoffset(); return; }
    if (key == "curl-drive controller adc")     { (void)cmd_con_adc(); return; }
    if (key == "curl-drive controller vel")      { (void)cmd_con_velocity(); return; }
    if (key == "curl-drive controller tmode")    { (void)cmd_con_tmode(); return; }
    if (key == "curl-drive controller mode")    { (void)cmd_con_mode(); return; }

    if (key == "curl-drive controller config")  { (void)cmd_con_config(flags); return; }
    if (key == "curl-drive controller writeconfig")  { (void)cmd_con_config_write(flags); return; }

    if (key == "curl-drive controller idle")  { (void)cmd_con_idle(); return; }
    if (key == "curl-drive controller torque")  { (void)cmd_con_torque(flags); return; }
    if (key == "curl-drive controller velocity")  { (void)cmd_con_velocity(flags); return; }
    if (key == "curl-drive controller position")  { (void)cmd_con_position(flags); return; }

    if (key == "curl-drive controller calibrate") { (void)cmd_con_calibrate(); return; }
    if (key == "curl-drive controller iscalibrated") { (void)cmd_con_iscalibrated(); return; }
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





