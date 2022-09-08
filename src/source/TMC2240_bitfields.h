#pragma once
#pragma pack(push, 1)

namespace TMC2240_n {
  struct GCONF_t {
    constexpr static uint8_t address = 0x00;
    union {
      uint16_t sr : 16;
      struct {
        bool    : 1,
              fast_standstill : 1,
              en_pwm_mode : 1,
              multistep_filt : 1,
              shaft : 1,
              diag0_error : 1,
              diag0_otpw : 1,
              diag0_stall : 1,
              diag1_stall : 1,
              diag1_index : 1,
              diag1_onstate : 1,
              diag0_pushpull : 1,            
              diag1_pushpull : 1,            
              small_hysteresis : 1,
              stop_enable : 1,
              direct_mode : 1;
      };
    };
  };
}

namespace TMC2240_n {
  struct IOIN_t {
    constexpr static uint8_t address = 0x04;
    union {
      uint32_t sr;
      struct {
        bool  step : 1,
              dir : 1,
              encb : 1,
              enca : 1,
              drv_enn : 1,
              encn : 1,
              uart_en : 1,
                : 1,
              comp_a : 1,
              comp_b : 1,
              comp_a1_a2 : 1,
              comp_b1_b2 : 1,
              output : 1,
              ext_res_det : 1,
              ext_clk : 1,
              adc_err : 1;
        uint8_t  silicon_rv : 3,
                : 5,
              version : 8;
      };
    };
  };
}


namespace TMC2240_n {
  struct DRV_CONF_t {
    constexpr static uint8_t address = 0x0A;
    union {
      uint32_t sr;
      struct {
        uint8_t current_range : 2,
                    : 2,
                slope_control : 2;
        uint32_t    : 16;
      };
    };
  };
};



namespace TMC2240_n {
  struct IHOLD_IRUN_t {
    constexpr static uint8_t address = 0x10;
    union {
      uint32_t sr;
      struct {
        uint16_t ihold : 5;
        uint8_t        : 3;
        uint8_t  irun  : 5,
                       : 3,
            iholddelay : 4,
                       : 4,
            irundelay  : 4;
      };
    };
  };
};



namespace TMC2240_n {
  struct CHOPCONF_t {
    constexpr static uint8_t address = 0x6C;
    union {
      uint32_t sr;
      struct {
        uint8_t toff : 4,
                hstrt : 3,
                hend : 4,
                    : 4,
                tbl : 2,
                fd3 : 1,
                disfdcc : 1,
                chm : 1,
                TBL : 2,
                    : 1,
                vhighfs : 1,
                vhighchm : 1,
                TPFD : 4, 
                MRES : 4, 
                intpol : 1, 
                dedge : 1,
                diss2g : 1,
                diss2vs : 1;
      };
    };
  };


  struct PWMCONF_t {
    constexpr static uint8_t address = 0x70;
    union {
      uint32_t sr;
      struct {
        uint8_t pwm_ofs : 8,
                pwm_grad : 8,
                pwm_freq : 2;
        bool    pwm_autoscale : 1,
                pwm_autograd : 1;
        uint8_t freewheel : 2,
                pwm_meas_sd_enable : 1,
                pwm_dis_reg_stst   : 1,
                pwm_reg : 4,
                pwm_lim : 4;
      };
    };
  };

  struct DRV_STATUS_t {
    constexpr static uint8_t address = 0x6F;
    union {
      uint32_t sr;
      struct {
        uint32_t SG_RESULT : 10;
        uint8_t            : 2; 
        uint8_t      s2vsa : 1,
                     s2vsb : 1,
                   stealth : 1,
                  fsactive : 1;
        uint16_t CS_ACTUAL : 5;
        uint8_t            : 3;
        bool    stallguard : 1,
                        ot : 1,
                      otpw : 1,
                      s2ga : 1,
                      s2gb : 1,
                       ola : 1,
                       olb : 1,
                      stst : 1;
      };
    };
  };

  struct PWM_SCALE_t {
    constexpr static uint8_t address = 0x71;
    union {
      uint32_t sr;
      struct {
        uint16_t pwm_scale_sum : 10,
                : 6;
        int16_t  pwm_scale_auto : 9;
      };
    };
  };


  struct PWM_AUTO_t {
    constexpr static uint8_t address = 0x72;
    union {
      uint32_t sr : 24;
      struct {
        uint8_t pwm_ofs_auto : 8,
                            : 8,
                pwm_grad_auto : 8;
      };
    };
  };
}

#pragma pack(pop)