#ifndef __DNP20_V0_DEF__
#define __DNP20_V0_DEF__


// Definition of Duth Network Protocol.
//   Interconnect's internal packetization protocol 
namespace dnp {
    enum {
      // 一个周期内可以并行传输的数据宽度
      PHIT_W = 24, // Phit Width
      
      V_W = 2, // Virtual Channel
      S_W = 4, // Source
      D_W = 4, // Destination
      Q_W = 3, // QoS
      T_W = 2, // Type，这里type包含四种类型：读请求、写请求、读响应、写响应，四种类型由PACK_TYPE枚举类定义
      
      // 指示传输属性所在位数
      V_PTR = 0,                // virtual channel在第零位
      S_PTR = (V_PTR + V_W),    // 源节点地址在第2位
      D_PTR = (S_PTR + S_W),    // 目的节点地址在第6位
      Q_PTR = (D_PTR + D_W),    // QoS信息在第10位
      T_PTR = (Q_PTR + Q_W),    // flit类型信息在第13位
      
      // AXI RELATED WIDTHS
      ID_W = 4, // AXI Transaction ID
      BU_W = 2, // AXI Burst，对应AxBURST[1:0]，用于指示突发类型
      SZ_W = 3, // AXI Size，对应AxSIZE[2:0]，表示一个transfer传输的数据大小
      LE_W = 8, // AXI Length，对应AxLEN[7:0],表示一次突发传输里包含的transfer的个数
      AL_W = 16, // Address Low
      AH_W = 16, // Address High
      AP_W = 8, // Address part (for alignment)
      RE_W = 2, // AXI Write Responce
      REORD_W = 3, // Ticket for reorder buffer
  
      B_W  = 8, // Byte Width，这里Byte Width实际上应该指的是一个LANE的宽度，即一个LANE可以传输8个bit
      E_W  = 1, // Enable width，这里是LANE选通信号，只需要一位就可以表示该LANE是否有效
      LA_W = 1, // AXI Last，LAST信号是用于指示当前transfer是事务最后一个transfer
    };
  
  // Read and Write Request field pointers
  struct req {
    enum {
      // T_PTR是基本传输属性中的最后一个属性，紧随其后的应该是实际请求或响应信息
      // 事务ID所在位置
      ID_PTR = T_PTR+T_W,
      // Ticket for reorder buffer
      REORD_PTR = ID_PTR+ID_W,
      
      // 读请求地址的低地址
      AL_PTR = 0,
      // AxLEN
      LE_PTR = AL_PTR+AL_W,
      
      // 读请求地址的高地址
      AH_PTR = 0,
      // AxSIZE
      SZ_PTR = AH_PTR+AH_W,
      // AxBURST
      BU_PTR = SZ_PTR+SZ_W,
    };
  };
  
  // Write Responce field pointers
  struct wresp {
    enum {
      ID_PTR    = T_PTR+T_W,
      REORD_PTR = ID_PTR+ID_W,
      RESP_PTR  = REORD_PTR+REORD_W,
    };
  };
  
  // Read Responce field pointers
  struct rresp {
    enum {
      ID_PTR    = T_PTR+T_W,
      REORD_PTR = ID_PTR+ID_W,
      // AxBURST
      BU_PTR    = REORD_PTR+REORD_W,
      
      // AxSIZE
      SZ_PTR = 0,
      // AxLEN
      LE_PTR = SZ_PTR+SZ_W,
      // Address part (for alignment)
      AP_PTR = LE_PTR+LE_W,
    };
  };
  
  // Write request Data field pointers
  struct wdata {
    // 因为一个PHIT只有24bit，而B_W为8，即一个LANE传输8bit，使能位又占去1bit，故一个phit里只能包含两个两个LANE传输来的数据
    enum {
      B0_PTR = 0,
      B1_PTR = B0_PTR+B_W,
      E0_PTR = B1_PTR+B_W,
      E1_PTR = E0_PTR+E_W,
      LA_PTR = E1_PTR+E_W,
    };
  };
  
  // Read response Data field pointers
  struct rdata {
    enum {
      B0_PTR = 0,
      B1_PTR = B0_PTR+B_W,
      RE_PTR = B1_PTR+B_W,
      LA_PTR = RE_PTR+RE_W,
    };
  };
  
  enum PACK_TYPE {
    PACK_TYPE__WR_REQ  = 0,
    PACK_TYPE__WR_RESP = 1,
    PACK_TYPE__RD_REQ  = 2,
    PACK_TYPE__RD_RESP = 3
  };
  
}

#endif // __DNP20_V0_DEF__
