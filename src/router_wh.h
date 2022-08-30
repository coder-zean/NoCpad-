#ifndef WH_ROUTER_CON_ST_BUF_H
#define WH_ROUTER_CON_ST_BUF_H

#include "systemc.h"
#include "./include/flit_axi.h"
#include "./include/duth_fun.h"
#include "./include/arbiters.h"

#include "nvhls_connections.h"

// Select In/Out Ports, the type of flit and the Routing Computation calculation function
// For RC_METHOD: 0-> direct rc 1-> lut, 2-> type, ... , 4-> LUT based routing
// IN_NUM    : Number of inputs
// OUT_NUM   : Number of outputs
// flit_t    : The networks flit type
// RC_METHOD : Routing Computation Algotrithm   路由计算算法
//               - 0 : Direct RC
//               - 1 : Constant RC    (for mergers)
//               - 2 : Packet type RC (for distinct routing of Writes and reads)
//               - 3 : For single stage NoCs
//               - 4 : LUT based RC
//               - 5 : XY routing with merged RD/WR Req-Resp
// DIM_X     : X Dimension of a 2-D mesh network. Used in XY routing
// NODES     : All possible target nodes of the network. Used in LUT routing
// ARB_C     : The arbiter type. Eg MATRIX, ROUND_ROBIN
template<unsigned int IN_NUM, unsigned int OUT_NUM, class flit_t, int RC_METHOD = 0, int DIM_X = 0, int NODES = 1, class ARB_C = arbiter<IN_NUM, MATRIX>>
SC_MODULE(router_wh_top) {
  
  // clog2用于计算一个数需要使用多少比特表示
  typedef sc_uint<clog2<OUT_NUM>::val> port_w_t;
  
  sc_in_clk    clk{"clk"};
	sc_in<bool>  rst_n{"rst_n"};
  
  // LUT is passed as a signal for LUT based RC. Otherwise Not-Used
  // 该项只有在路由计算算法为LUT路由算法时才有用
	sc_in<sc_uint<dnp::D_W>> route_lut[NODES];
  
  // id_x and id_y are the X,Y dimensions of the router in a 2-D mesh network. Otherwise Not-Used 
  sc_in<sc_uint<dnp::D_W>>  id_x{"id_x"};
	sc_in<sc_uint<dnp::D_W>>  id_y{"id_y"};
	
  // Input channels
  Connections::InBuffered <flit_t, 2>  data_in[IN_NUM];
  // Output channels
  Connections::OutBuffered<flit_t, 1>  data_out[OUT_NUM];
  
  // Per input
  // Each input has a lock bit, meaning the required outport has been locked for this input
  bool out_lock[IN_NUM];
  // Each input stores its required outport (for body/tail flits)
  port_w_t out_port[IN_NUM];
  
  // Per Output 
  // out available holds the availability of the corresponding output port
  // out available保存对应输出端口的可用性
  bool   out_available[OUT_NUM];
  ARB_C  arbiter[OUT_NUM];
  
  // Constructor
  SC_HAS_PROCESS(router_wh_top);
  router_wh_top(sc_module_name name_="router_wh_top")
    : sc_module(name_)
  { 
    SC_THREAD(router_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
  
  void router_job (){
  #pragma hls_unroll yes
  per_i_rst : for (unsigned char i = 0; i < IN_NUM; ++i) {
      data_in[i].Reset();
      out_lock[i] = false;
      out_port[i] = 0;
    }
  #pragma hls_unroll yes
  per_o_rst : for (unsigned char o = 0; o < OUT_NUM; ++o) {
      data_out[o].Reset();
      out_available[o] = true;
    }
    
    // Post Reset
    #pragma hls_pipeline_init_interval 1
    #pragma pipeline_stall_mode flush
    while (1) {
      wait();
      bool   fifo_valid[IN_NUM];
      flit_t hol_data[IN_NUM];
      
      // The request and grants of the Inputs/Outputs
      bool qualified_reqs[OUT_NUM][IN_NUM];
      sc_uint<OUT_NUM> req_per_i[IN_NUM];
      sc_uint<IN_NUM>  req_per_o[OUT_NUM];
      
      sc_uint<IN_NUM>  gnt_per_o[OUT_NUM];
      sc_uint<OUT_NUM> gnt_per_i[IN_NUM];
      
      bool is_inp_granted[IN_NUM][OUT_NUM];
      
      // Input logic, loops for each input to produce the required requests
      #pragma hls_unroll yes
      set_inp: for (int ip = 0; ip < IN_NUM; ++ip) {
        // The input checks for available data to sent.
        if (data_in[ip].Empty()) {
          // 若对应输入端口为空，则将其合法位置为false，并用一个空flit填充到hol_data中
          fifo_valid[ip] = false;
          hol_data[ip] = flit_t();
        } else {
          // 若对应输入端口不为空，则将其对应合法位置为true，并获取端口的第一个flit填充到hol_data中
          fifo_valid[ip] = true;
          hol_data[ip] = data_in[ip].Peek();
        }
    
        // Depending the Flit type the input selects an output port to request.
        // 根据输入flit的类型，选择要请求的输出端口
        // The required output gets stored to be used by the rest of the flits.
        // 所需输出端口会被存储起来，供后续的flit继续使用
        port_w_t  current_op;
        // performs_rc方法用于判断该flit是否为header或者single
        bool      is_head_single = hol_data[ip].performs_rc();

        // 如果是head或者single flit，则需要进行路由选择计算出输出端口；如果是body或者tail flit，则可以沿用之前已经保存的输出端口
        if (fifo_valid[ip] && is_head_single) {
          // Route Computation Methods
          if      (RC_METHOD == 0) { current_op = do_rc_direct(hol_data[ip].get_dst());} // returns the node ID
          else if (RC_METHOD == 1) { current_op = do_rc_const();} // returns 0. Used for mergers
          else if (RC_METHOD == 2) { current_op = do_rc_type(hol_data[ip].get_type());} // Return 0/1 depending the type. used for splitters
          else if (RC_METHOD == 3) { current_op = do_rc_common(hol_data[ip].get_dst(), hol_data[ip].get_type());}
          else if (RC_METHOD == 4) { current_op = do_rc_lut(hol_data[ip].get_dst());}
          else if (RC_METHOD == 5) { current_op = do_rc_xy_merge(hol_data[ip].get_dst(), hol_data[ip].get_type());}
          else                     { NVHLS_ASSERT_MSG(0, "Wrong Routing method selected.");}
          
          out_port[ip] = current_op;
        } else {
          current_op = out_port[ip];
        }
        
        // The required output port must be also Ready and or available.
        sc_uint<OUT_NUM> port_req_oh = (1 << current_op); // wb2oh_case<OUT_NUM>(current_op);// (1<<current_op);
        bool ready_outp[OUT_NUM];
        #pragma hls_unroll yes
        // 将已经full的输出端口置为false，表示不能再放入数据；将还没有full的输出端口置为true，表示输出端口ready
        for (int op = 0; op < OUT_NUM; ++op) ready_outp[op] = !data_out[op].Full();
  
        bool outp_ready = mux<bool, OUT_NUM>::mux_oh_case(port_req_oh, ready_outp);
        bool outp_avail = mux<bool, OUT_NUM>::mux_oh_case(port_req_oh, out_available);
        
        bool all_ok = (fifo_valid[ip] && outp_ready && (out_lock[ip] || (is_head_single && outp_avail)));
        req_per_i[ip] = all_ok ? port_req_oh : (sc_uint<OUT_NUM>) 0;
      } // End of set_inp
      
      swap_dim<sc_uint<OUT_NUM>, IN_NUM, sc_uint<IN_NUM>, OUT_NUM>(req_per_i, req_per_o);
      
      // Loop each of the outputs, choosing an input to win the output using an arbitration scheme
      #pragma hls_unroll yes
      per_o : for (unsigned char op = 0; op < OUT_NUM; ++op) {
        bool      any_gnt; // the output has been granted
        port_w_t  gnt_ip;  // Input port that got grant
  
        any_gnt = arbiter[op].arbitrate(req_per_o[op], gnt_per_o[op]);
        
        flit_t selected_flit;
        selected_flit = mux<flit_t, IN_NUM>::mux_oh_case(gnt_per_o[op], hol_data);
        if (any_gnt) {
          data_out[op].Push(selected_flit);
          
          if      (selected_flit.is_head()) out_available[op] = false;
          else if (selected_flit.is_tail()) out_available[op] = true;
        }
      } // End per_o
  
      swap_dim<sc_uint<IN_NUM>, OUT_NUM, sc_uint<OUT_NUM>, IN_NUM>(gnt_per_o, gnt_per_i);
      
      // Loop through each input to handle the case of actually winning the output
      #pragma hls_unroll yes
      popped_i : for (unsigned char ip = 0; ip < IN_NUM; ++ip){
        if (gnt_per_i[ip].or_reduce()) {
          // Update the local lock bit depending the flit type
          //   Head->locks Tail->unlocks
          if      (hol_data[ip].is_head()) out_lock[ip] = true;
          else if (hol_data[ip].is_tail()) out_lock[ip] = false;
          
          data_in[ip].Pop();
        }
      }
      
      
      // Move flits from internal Buffer to Out Port
      #pragma hls_unroll yes
      for (unsigned char op = 0; op < OUT_NUM; ++op)
        data_out[op].TransferNB();
      // Transfer any flit at input Port to its internal buffer (This is due to InBuffered)
  
      // Move flits from In Port to internal Buffer
      #pragma hls_unroll yes
      for (unsigned char ip = 0; ip < IN_NUM; ++ip) {
        data_in[ip].TransferNB();
      }
    }
  }; // End of Router Job
  
  
  // Direct RC : The Dst Node is the Output port
  inline unsigned char do_rc_direct (sc_lv<dnp::D_W> destination) {return destination.to_uint();};
  // TYPE RC : Used for splitter/mergers. Routes RD to Out==0, and WR to Out==1
  inline unsigned char do_rc_type   (sc_lv<dnp::T_W> type) {return (type==dnp::PACK_TYPE__RD_REQ || type==dnp::PACK_TYPE__RD_RESP) ? 0 : 1;};
  // Const RC : Used for mergers to always request port #0
  inline unsigned char do_rc_const  () {return 0;};
  // For single stage NoCs
  inline unsigned char do_rc_common  (sc_lv<dnp::D_W> destination, sc_lv<dnp::T_W> type) {
    if      (type == dnp::PACK_TYPE__RD_REQ)  return destination.to_uint();
    else if (type == dnp::PACK_TYPE__RD_RESP) return destination.to_uint()-2;
    else if (type == dnp::PACK_TYPE__WR_REQ)  return destination.to_uint()+2;
    else                                      return destination.to_uint();
  };
  // LUT Based RC
  inline unsigned char do_rc_lut (sc_lv<dnp::D_W> destination) {
    return route_lut[destination.to_uint()].read();
  };
  // XY merged RD/WR
  inline unsigned char do_rc_xy_merge (sc_uint<dnp::D_W> destination, sc_uint<dnp::T_W> type) {
    sc_uint<dnp::D_W> this_id_x = id_x.read();
    sc_uint<dnp::D_W> this_id_y = id_y.read();
  
    sc_uint<dnp::D_W> dst_x = destination % DIM_X;
    sc_uint<dnp::D_W> dst_y = destination / DIM_X;
    
    if (dst_x > this_id_x) {
      return 1;
    } else if (dst_x < this_id_x) {
      return 0;
    } else {
      if (dst_y > this_id_y) {
        return 3;
      } else if (dst_y < this_id_y) {
        return 2;
      } else {
        if      (type == dnp::PACK_TYPE__RD_REQ)  return 4;
        else if (type == dnp::PACK_TYPE__RD_RESP) return 4;
        else if (type == dnp::PACK_TYPE__WR_REQ)  return 5;
        else                                      return 5;
      }
    }
  };
  
};

#endif // WH_ROUTER_CON_ST_BUF_H
