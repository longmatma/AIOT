#ifndef V2C_NODE_CONFIG_H
#define V2C_NODE_CONFIG_H

// =====================================================
// V2C1 DUAL-PAIR NODE CONFIG
// Build cùng source cho 2 SU. Chỉ đổi V2C_NODE_INDEX trước khi upload.
//   SU1/DU1 = 0x01/0x02
//   SU2/DU2 = 0x04/0x05
//   rBS     = 0x03
// =====================================================
#ifndef V2C_NODE_INDEX
#define V2C_NODE_INDEX 1
#endif

#define V2C_RBS_ID       0x03
#define V2C_BROADCAST_ID 0xFF

#if V2C_NODE_INDEX == 1
  #define V2C_PAIR_INDEX                 1
  #define V2C_SU_ID                      0x01
  #define V2C_DU_ID                      0x02
  #define V2C_UL_VOICE_OFFSET_MS         12U
  #define V2C_UL_LATEST_START_MS         42U
  #define V2C_BOOTSTRAP_SESSION_DELAY_MS 0U
#elif V2C_NODE_INDEX == 2
  #define V2C_PAIR_INDEX                 2
  #define V2C_SU_ID                      0x04
  #define V2C_DU_ID                      0x05
  #define V2C_UL_VOICE_OFFSET_MS         115U
  #define V2C_UL_LATEST_START_MS         145U
  #define V2C_BOOTSTRAP_SESSION_DELAY_MS 90U
#else
  #error "V2C_NODE_INDEX phai la 1 hoac 2"
#endif

// Dedicated parity slot. Chỉ node được beacon grant mới được TX FEC.
#define V2C_FEC_SLOT_OFFSET_MS 215U

#endif
