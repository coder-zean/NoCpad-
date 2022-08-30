#ifndef AXI_MASTER_H
#define AXI_MASTER_H

#include "systemc.h"

#include <axi/axi4.h>
#include "../helper_non_synth.h"
#include "../../src/include/flit_axi.h"
#include "../tb_wrap.h"

#include <deque>
#include <queue>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


#define AXI_TID_NUM 4
#define AXI_BURST_NUM 3

// question：AXI4_MAX_INCR_LEN应该比AXI4_MAX_LEN大，但是这里定义都为4
#define AXI4_MAX_LEN      4     // FIXED, WRAP bursts has a maximum of 16 beats
#define AXI4_MAX_INCR_LEN 4     // AXI4 extends INCR bursts upto 256 beats


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
SC_MODULE(axi_master) {
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  typedef typename axi::AXI4_Encoding                 enc_;
  
	sc_in_clk    clk;
  sc_in<bool>  rst_n;

	sc_in<bool>  stop_gen;
    
  sc_in<sc_uint<32>>  addr_map[SLAVE_NUM][2];
  
	Connections::Out<axi4_::AddrPayload>   ar_out;
	Connections::In<axi4_::ReadPayload>    r_in;
  
	Connections::Out<axi4_::AddrPayload>   aw_out;
	Connections::Out<axi4_::WritePayload>  w_out;
	Connections::In<axi4_::WRespPayload>   b_in;
  
  // Scoreboard
  sc_mutex                                                    *sb_lock;
  // 为每一个目的slave都维护一个deque用于存放事务请求
  // 为每一个master都维护一个expect response队列
  std::vector<std::deque<msg_tb_wrap<axi4_::AddrPayload>>>    *sb_rd_req_q;
  std::vector<std::deque<msg_tb_wrap<axi4_::ReadPayload>>>    *sb_rd_resp_q;
  
  std::vector<std::deque<msg_tb_wrap<axi4_::AddrPayload>>>    *sb_wr_req_q;
  std::vector<std::deque<msg_tb_wrap<axi4_::WritePayload>>>   *sb_wr_data_q;
  std::vector<std::deque<msg_tb_wrap<axi4_::WRespPayload>>>   *sb_wr_resp_q;
  
  std::deque<axi4_::AddrPayload>  sb_rd_order_q; // queue to check order
  std::deque<axi4_::AddrPayload>  sb_wr_order_q; // queue to check order
  
  // 产生的读写请求和写数据放在以下队列中，等待程序将其调度并push出去
  std::queue<axi4_::AddrPayload>   stored_rd_trans;
  std::queue<axi4_::AddrPayload>   stored_wr_trans;
  std::queue<axi4_::WritePayload>  stored_wr_data;
  
	int MASTER_ID  = -1;
  // 用于控制读事务生成速度
	unsigned int GEN_RATE_RD;
  // 用于控制写事务生成速度
  unsigned int GEN_RATE_WR;
  // int FLOW_CTRL;     // 0: READY-VALID
  //                    // 1: CREDITS 
  //                    // 2: FIFO
  //                    // 3: Credits fifo based
  
  // delays
	int total_cycles;
  sc_time clk_period;
  
  unsigned long long int rd_resp_delay = 0;
  // xx_resp_count记录的是对整个事务响应的个数
  unsigned long long int rd_resp_count = 0;
  unsigned long long int wr_resp_delay = 0;
  unsigned long long int wr_resp_count = 0;
  unsigned long long int last_rd_sinked_cycle = 0;
  unsigned long long int last_wr_sinked_cycle = 0;
  // xx_resp_data_count记录的是总共发送了多少个transfer
  unsigned long long int rd_resp_data_count = 0;
  unsigned long long int wr_resp_data_count = 0;
  
	bool stop_at_tail, has_stopped_gen;
  
  // Read Addr Generator
  // 分别用于记录读写事务和读写数据产生的个数
  int rd_trans_generated;
  int rd_data_generated;
  int wr_trans_generated;
  int wr_data_generated;
  
  // 分别用于记录已经发送的读写事务请求和写数据个数
  int rd_trans_inj;
  int wr_trans_inj;
  int wr_data_inj;
  
  unsigned int gen_rd_addr;
  unsigned int gen_wr_addr;
  unsigned int resp_val_expect;
  
  // Read Responce Sink
  // 用于记录收到读写响应的个数，这里读响应是针对transfer的
  int rd_resp_ej;
  int wr_resp_ej;
  
  // Errors
  int error_sb_rd_resp_not_found;
  int error_sb_wr_resp_not_found;
  
  // Functions
	void do_cycle();
	void gen_new_rd_trans();
	void gen_new_wr_trans();
  
	void verify_rd_resp(axi4_::ReadPayload  &rcv_rd_resp);
	void verify_wr_resp(axi4_::WRespPayload &rcv_wr_resp);
  
  bool eq_rd_data(axi4_::ReadPayload &rcv_rd_data, axi4_::ReadPayload &sb_rd_data);
  bool eq_wr_resp(axi4_::WRespPayload &rcv_wr_resp, axi4_::WRespPayload &sb_wr_resp);
  
  unsigned mem_map_resolve(axi4_::Addr &addr);
  
  // Constructor
  SC_HAS_PROCESS(axi_master);
  axi_master(sc_module_name name_) : sc_module(name_)
  {
    MASTER_ID = -1;
    GEN_RATE_RD  = 0;
    GEN_RATE_WR  = 0;
    
		SC_THREAD(do_cycle);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
};


// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- IMPLEMENTATION  --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //

#include "axi_master.h"

// --------------------------- //
// ------ DO CYCLE      ------ //
// --------------------------- //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::do_cycle () {
  total_cycles       = 0;
  
  rd_trans_generated = 0;
  rd_data_generated  = 0;
  wr_trans_generated = 0;
  wr_data_generated  = 0;
  rd_trans_inj       = 0;
  wr_trans_inj       = 0;
  wr_data_inj        = 0;
  gen_rd_addr        = 0;
  gen_wr_addr        = 0;
  resp_val_expect    = 0;
  
  rd_resp_ej = 0;
  wr_resp_ej = 0;
  
  // Clear errors
  error_sb_rd_resp_not_found = 0;
  error_sb_wr_resp_not_found = 0;
  
  clk_period = (dynamic_cast<sc_clock *>(clk.get_interface()))->period();
  
  while(1) {
    wait();
    
    // Transaction Generator
    if (!stop_gen.read()) {
      unsigned int rnd_val_rd = rand() % 100;
      // 这里GEN_RATE_RD和GEN_RATE_WR都是0，很奇怪，这样将永远不会生成新事物；除非rand()可以返回负数，但是查看百度，rand只能返回大于0以上的数
      if (rnd_val_rd < GEN_RATE_RD) {
        gen_new_rd_trans();
      }
      
      unsigned int rnd_val_wr = rand() % 100;
      if (rnd_val_wr < GEN_RATE_WR) {
        gen_new_wr_trans();
      }
    }
    
    // Read Request Injection
    // 判断读请求事务产生队列是否为空，若不为空，从队列中取出一个事务，并将其push到ar_out中
    if (!stored_rd_trans.empty()) {
      axi4_::AddrPayload tmp_ar = stored_rd_trans.front();
      if (ar_out.PushNB(tmp_ar)){
        stored_rd_trans.pop();
        std::cout << "[Master " << MASTER_ID << "] : PUSHED AR:" << tmp_ar << " @" << sc_time_stamp() << std::endl;
        rd_trans_inj++;
      }
    }
    
    // Write Request Injection
    if (!stored_wr_trans.empty()) {
      axi4_::AddrPayload tmp_aw = stored_wr_trans.front();
      if (aw_out.PushNB(tmp_aw)) {
        stored_wr_trans.pop();
        std::cout << "[Master "<< MASTER_ID << "] : PUSHED AW: " << tmp_aw << " @" << sc_time_stamp() << std::endl;
        wr_trans_inj++;
      }
    }
    
    // Write Data Injection
    if (!stored_wr_data.empty()) {
      axi4_::WritePayload tmp_w = stored_wr_data.front();
      if (w_out.PushNB(tmp_w)) {
        stored_wr_data.pop();
        std::cout << "[Master "<< MASTER_ID << "] : PUSHED W: " << tmp_w << " @" << sc_time_stamp() << std::endl;
        wr_data_inj++;
      }
    }
    
    // Read Response Ejection
    // 从r_in中获取读响应
    axi4_::ReadPayload rcv_rd_resp;
    if (r_in.PopNB(rcv_rd_resp)) {
      verify_rd_resp(rcv_rd_resp);
    }

    // Write Response Ejection
    axi4_::WRespPayload rcv_wr_resp;
    if(b_in.PopNB(rcv_wr_resp)) {
      verify_wr_resp(rcv_wr_resp);
    }
    
  }; // End of while(1)
}; // End of do_cycle

// --------------------------- //
// --- GENERATOR Functions --- //
// --------------------------- //
// 生成读事务
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_rd_trans() {
  axi4_::AddrPayload rd_req_m;
  
  // AXI_TID_NUM == 4，所以读事务id范围为0 - 3
  rd_req_m.id    = (rand() % AXI_TID_NUM);
  // 1 << size小于RD_M_LANES
  rd_req_m.size  = ((rand() % my_log2c(RD_M_LANES)) + 1) & ((1 << my_log2c(RD_M_LANES)) - 1);
  rd_req_m.burst = (rand() % AXI_BURST_NUM);
  // question：rand() % AXI4_MAX_LEN最大只有3
  // WRAP burst的传输长度只能是2,4
  rd_req_m.len   = (rd_req_m.burst == enc_::AXBURST::WRAP)  ? ((1 << (rand() % my_log2c(AXI4_MAX_LEN + 1))) - 1)        :
                   (rd_req_m.burst == enc_::AXBURST::FIXED) ? (rand() % AXI4_MAX_LEN)                                   :
                   (RD_M_LANES > RD_S_LANES)                ? (rand() % (AXI4_MAX_INCR_LEN / (RD_M_LANES / RD_S_LANES))):    // INCR With    Downsize // Cap the maximum len in case of transactions downsize (which increases len)
                   (rand() % AXI4_MAX_INCR_LEN);                                                                              // INCR WithOut Downsize
  
  // Increasing address to keep track of the transactions
  rd_req_m.addr  = gen_rd_addr + ((rand() % RD_M_LANES) & (1 << rd_req_m.size));
  
  rd_req_m.addr  = (rand() % 2) ? gen_rd_addr : gen_rd_addr + 0x10000; // addr_map[i][1].read();
  gen_rd_addr = gen_rd_addr + RD_M_LANES;
  
  // Push it to injection queue
  // 将产生的请求放入存储队列中，等待调度
  stored_rd_trans.push(rd_req_m);
  
  // Push it to Scoreboard
  sb_lock->lock();

  // Consider resizing at slave
  // 判断是否需要在从机将事务进行拆分
  axi4_::AddrPayload rd_req_s;
  rd_req_s.id    = rd_req_m.id;
  // 若master读事务请求的size大于slave的数据通道，则将size变为log2(RD_S_LANES)向下取整；反之则不变
  rd_req_s.size  = ((1 << rd_req_m.size) > RD_S_LANES) ? my_log2c(RD_S_LANES) : (unsigned)rd_msg_tb_wrapreq_m.size;
  // 若size改变了，则len也需要相应改变
  rd_req_s.len   = ((1 << rd_req_m.size) > RD_S_LANES) ? unsigned(((rd_req_m.len + 1) << (rd_req_m.size - my_log2c(RD_S_LANES))) - 1) : (unsigned)rd_req_m.len;
  rd_req_s.burst = rd_req_m.burst;
  rd_req_s.addr  = rd_req_m.addr;
  
  <axi4_::AddrPayload> temp_rd_req_tb;
  temp_rd_req_tb.dut_msg = rd_req_s;
  
  // 记录请求产生时间
  temp_rd_req_tb.time_gen = sc_time_stamp();
  
  // 解析请求目的地址是那个slave
  unsigned dst = mem_map_resolve(rd_req_s.addr);
  // 将信息放到对应slave的scoreboard中
  (*sb_rd_req_q)[dst].push_back(temp_rd_req_tb);
  sb_lock->unlock();
  
  // Push into order queue - Reorder check extension
  sb_rd_order_q.push_back(rd_req_m);
  
  rd_trans_generated++;
  
  // --- --- --- --- --- --- --- --- //
  // Generate the expected Responce
  // --- --- --- --- --- --- --- --- //
  sb_lock->lock();
  axi4_::ReadPayload beat_expected;
  
  beat_expected.id = rd_req_m.id;
  
  // Create Expected Response
  unsigned long int bytes_total = ((rd_req_m.len + 1) << rd_req_m.size);
  unsigned long int byte_count  = 0;
  
  // m_init_ptr是用于处理非对齐地址的
  unsigned char m_init_ptr  = rd_req_m.addr % RD_M_LANES;
  unsigned char m_ptr       = m_init_ptr;
  unsigned char m_size      = rd_req_m.size;
  
  beat_expected.data = 0;
  
  while (byte_count < bytes_total) {
    beat_expected.data |= (((axi4_::Data)(byte_count & 0xFF)) << ((axi4_::Data)(m_ptr * 8)));
    byte_count++;

    // FIXED读取地址不变，故每次都需要处理非对齐问题；其余burst type只需要在第一次读取时处理非对齐问题，之后的transfer都无需关心非对齐问题
    m_ptr = (rd_req_m.burst == enc_::AXBURST::FIXED) ? ((m_ptr + 1) % (1 << m_size)) + m_init_ptr
                                                     : (m_ptr + 1) % RD_M_LANES ;
    
    if (((m_ptr % (1 << m_size)) == 0) || (byte_count == bytes_total)) {
      beat_expected.resp = mem_map_resolve(rd_req_m.addr);
      beat_expected.last = (byte_count == bytes_total);
      
      msg_tb_wrap<axi4_::ReadPayload> temp_rd_resp_tb;
      temp_rd_resp_tb.dut_msg  = beat_expected;
      temp_rd_resp_tb.time_gen = sc_time_stamp();
      
      (*sb_rd_resp_q)[MASTER_ID].push_back(temp_rd_resp_tb);
      beat_expected.data = 0;

      rd_data_generated++;
    }
  }
  sb_lock->unlock();
}; // End of Read generator

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_wr_trans() {
  sb_lock->lock();
  axi4_::AddrPayload m_wr_req;
  
  m_wr_req.id    = (rand()%AXI_TID_NUM);
  m_wr_req.size  = ((rand()%my_log2c(WR_M_LANES))+1) & ((1<<my_log2c(WR_M_LANES))-1); // 0 size is NOT supported
  m_wr_req.burst = (rand()%AXI_BURST_NUM);
  m_wr_req.len   = (m_wr_req.burst==enc_::AXBURST::WRAP)  ? ((1<<(rand()%my_log2c(AXI4_MAX_LEN+1)))-1)           :
                   (m_wr_req.burst==enc_::AXBURST::FIXED) ? (rand()%AXI4_MAX_LEN)                                :
                   (WR_M_LANES>WR_S_LANES) ? (rand()%(AXI4_MAX_INCR_LEN/(WR_M_LANES/WR_S_LANES)))    // INCR With    Downsize // Cap the maximum len in case of transactions downsize (which increases len)
                                           : (rand()%AXI4_MAX_INCR_LEN)                           ;  // INCR WithOut Downsize
  
  // Increasing address to keep track of the transactions
  // Aligned on size transactions (Although non-aligned should be an easy addition)
  m_wr_req.addr  = gen_wr_addr + ((rand()%WR_M_LANES) & (1<<m_wr_req.size));
  
  m_wr_req.addr   = (rand() % 2) ? gen_wr_addr : gen_wr_addr + 0x10000; //addr_map[i][1].read();
  gen_wr_addr    = gen_wr_addr + WR_M_LANES;
  
  // Push it to injection queue
  stored_wr_trans.push(m_wr_req);
  
  // Push into order queue - Reorder check extension
  sb_wr_order_q.push_back(m_wr_req);
  
  // Create dummy write data
  axi4_::WritePayload cur_beat;      // The beat that will be injected at MASTER
  axi4_::WritePayload beat_at_slave; // The expected beat ejected at SLAVE
  
  unsigned long int bytes_total = ((m_wr_req.len+1)<<m_wr_req.size);
  unsigned long int byte_count  = 0;
  
  unsigned char m_init_ptr = m_wr_req.addr % WR_M_LANES;
  unsigned char s_init_ptr = m_wr_req.addr % WR_S_LANES;
  
  unsigned char m_ptr = m_init_ptr;
  unsigned char s_ptr = s_init_ptr;
  
  unsigned char m_size = m_wr_req.size;
  unsigned char s_size = ((1<<m_size)>WR_S_LANES) ? my_log2c(WR_S_LANES) : m_size;

  unsigned char m_len = m_wr_req.len;
  unsigned char s_len = ((1<<m_size)>WR_S_LANES) ? (((m_len+1)<<(m_size-s_size))-1) : m_len;
  
  // Push it to Scoreboard
  axi4_::AddrPayload s_wr_req;
  s_wr_req.id   = m_wr_req.id;
  s_wr_req.addr  = m_wr_req.addr;
  s_wr_req.size  = s_size;
  s_wr_req.len   = s_len;
  s_wr_req.burst = m_wr_req.burst;
  
  msg_tb_wrap<axi4_::AddrPayload> temp_wr_req_tb;
  temp_wr_req_tb.dut_msg = s_wr_req;
  
  unsigned dst = mem_map_resolve(s_wr_req.addr);
  (*sb_wr_req_q)[dst].push_back(temp_wr_req_tb);
  
  cur_beat.data  = 0;
  cur_beat.wstrb = 0;
  beat_at_slave.data  = 0;
  beat_at_slave.wstrb = 0;
  
  while(byte_count<bytes_total) {
    unsigned byte_to_write = (byte_count==bytes_total-1) ? MASTER_ID : byte_count;
    cur_beat.data |= (((axi4_::Data)(byte_to_write & 0xFF)) << ((axi4_::Data)(m_ptr*8)));
    cur_beat.wstrb |= (((axi4_::Data)1) << ((axi4_::Data)m_ptr));
    
    beat_at_slave.data  |= (((axi4_::Data)(byte_to_write & 0xFF)) << ((axi4_::Data)(s_ptr*8)));
    beat_at_slave.wstrb |= (((axi4_::Data)1) << ((axi4_::Data)s_ptr));
    
    byte_count++;
  
    m_ptr = (m_wr_req.burst==enc_::AXBURST::FIXED) ? ((m_ptr+1)%(1<<m_size)) + m_init_ptr :
                                                     (m_ptr+1)%WR_M_LANES ;
    
    s_ptr = (m_wr_req.burst==enc_::AXBURST::FIXED) ? ((s_ptr+1)%(1<<s_size)) + s_init_ptr :
                                                     (s_ptr+1)%WR_S_LANES ;
    
    if(((m_ptr%(1<<m_size))==0) || (byte_count == bytes_total)) {
      wr_data_generated++;
      cur_beat.last = (byte_count == bytes_total);
      stored_wr_data.push(cur_beat);
      cur_beat.data  = 0;
      cur_beat.wstrb = 0;
    }

    if(((s_ptr%(1<<s_size))==0) || (byte_count == bytes_total)) {
      beat_at_slave.last = (byte_count == bytes_total);
      msg_tb_wrap< axi4_::WritePayload > temp_wr_data_tb;
      temp_wr_data_tb.dut_msg = beat_at_slave;
        
      wr_resp_data_count++;
      last_wr_sinked_cycle = (sc_time_stamp() / clk_period);
      
      (*sb_wr_data_q)[dst].push_back(temp_wr_data_tb);
      beat_at_slave.data  = 0;
      beat_at_slave.wstrb = 0;
    }
  }
  
  sb_lock->unlock();
  
  wr_trans_generated++;
}; // End of Read generator

// ------------------------ //
// --- VERIFY Functions --- //
// ------------------------ //
// 这里的读响应是每个transfer都会触发的
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_rd_resp(axi4_::ReadPayload &rcv_rd_resp){
  // Verify Responce
  sb_lock->lock();
  
  // --- Reorder Check --- //
  int reorder = 2; // 2 : Req not found, 1 : Request reordered, 0 : everything is fine
  unsigned j  = 0;
  axi4_::AddrPayload sb_ord_req;
  
  // 遍历请求reorder列表，查看是否有请求id和响应id相同的请求事务，若无，则reorder字段为2，表示没有找到对应请求
  while (j < sb_rd_order_q.size()){
    sb_ord_req = sb_rd_order_q[j];
    
    // 判断请求id和响应id是否相同
    if (sb_ord_req.id == rcv_rd_resp.id) {
      // Slave must sneak its ID to the resp field.
      unsigned dst = mem_map_resolve(sb_ord_req.addr);
      // 如果请求地址所属slave和返回响应的slave相同，则表示everything is fine；否则则表示Request reordered
      // question:这里dst != rcv_rd_resp.resp时代表什么？猜测应该是事务已经被完成修改合并
      reorder = (dst == rcv_rd_resp.resp) ? 0 : 1;
      // 如果获得的响应transfer已经是读事务的最后的transfer，则从读请求的reorder队列中删除
      if (rcv_rd_resp.last) sb_rd_order_q.erase(sb_rd_order_q.begin() + j);
      break;
    }
    j++;
  }
  // --------------------- //
  
  bool found = false;
  j = 0;
  // 遍历主机的期望响应，从中查看是否有和收到的响应相同的响应
  while (j < (*sb_rd_resp_q)[MASTER_ID].size()){
    msg_tb_wrap<axi4_::ReadPayload> sb_resp = (*sb_rd_resp_q)[MASTER_ID][j];
    
    // 判断收到的响应和主机生成的期望响应是否相同
    if (eq_rd_data(rcv_rd_resp, sb_resp.dut_msg)){
      if (sb_resp.dut_msg.last) {
        rd_resp_delay += ((sc_time_stamp() - sb_resp.time_gen) / clk_period) - 1;
        // 这里的rd_resp记录的是整个事务的响应，而不是transfer的响应
        rd_resp_count++; 
      }
      // resp_data记录的是读数据transfer的个数
      rd_resp_data_count++;
      last_rd_sinked_cycle = (sc_time_stamp() / clk_period);
      
      (*sb_rd_resp_q)[MASTER_ID].erase((*sb_rd_resp_q)[MASTER_ID].begin() + j);
      found = true;
      break;
    }
    j++;
  }
  
  if (!found) {
    std::cout << "\n\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "-SB_front - "<< (*sb_rd_resp_q)[MASTER_ID].front() << "\n";
    error_sb_rd_resp_not_found++;
    sc_assert(0);
    // sc_stop();
  } else if (reorder == 2) {
    std::cout << "\n\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . Respective Request wasn't found!!! @" << sc_time_stamp() << "\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "-REQ_front - "<< sb_rd_order_q.front() << "\n";
    sc_assert(0);
  } else if (reorder == 1) {
    std::cout << "\n\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . Got Reordered !!! @" << sc_time_stamp() << "\n";
    std::cout << "[Master " << MASTER_ID <<"] " << "REQ-Ordered - "<< sb_ord_req << "\n";
    sc_assert(0);
  } else{
    std::cout<< "[Master " << MASTER_ID <<"] " << "RD-Resp OK   : <<  " << rcv_rd_resp << " @" << sc_time_stamp() << "\n";
    // 收到读响应的记录数++
    rd_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of READ Response Verify


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_wr_resp(axi4_::WRespPayload &rcv_wr_resp){
  // Verify Responce
  sb_lock->lock();
  // --- Reorder Check --- //
  int reorder=2; // 2 : Req not found, 1 : Request reordered, 0 : everything is fine
  unsigned j=0;
  axi4_::AddrPayload sb_ord_req;
  while (j<sb_wr_order_q.size()){
    sb_ord_req = sb_wr_order_q[j];
    
    if(sb_ord_req.id == rcv_wr_resp.id) {
      // Slave must sneak its ID into the first data byte of every beat (aka data[0]).
      unsigned dst = mem_map_resolve(sb_ord_req.addr);
      reorder = (dst  == rcv_wr_resp.resp) ? 0 : 1;
      sb_wr_order_q.erase(sb_wr_order_q.begin()+j);
      break;
    }
    j++;
  }
  // --------------------- //
  
  // Verify Responce
  bool found=false;
  j=0;
  while (j<(*sb_wr_resp_q)[MASTER_ID].size()){
    msg_tb_wrap< axi4_::WRespPayload > sb_resp = (*sb_wr_resp_q)[MASTER_ID][j];
    
    if ( eq_wr_resp(sb_resp.dut_msg, rcv_wr_resp) ){
      
      wr_resp_delay += ((sc_time_stamp() - sb_resp.time_gen) / clk_period) - 1;
      wr_resp_count++;
      
      (*sb_wr_resp_q)[MASTER_ID].erase((*sb_wr_resp_q)[MASTER_ID].begin()+j);
      found = true;
      break;
    }
    j++;
  }
  
  if(!found){
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-SB_front - "<< (*sb_wr_resp_q)[MASTER_ID].front() << "\n";
    error_sb_wr_resp_not_found++;
    sc_assert(0);
    // sc_stop();
  }else if(reorder==2) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . Respective Request wasn't found!!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-REQ_front - "<< sb_wr_order_q.front() << "\n";
    sc_assert(0);
  }else if(reorder==1) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . Got Reordered !!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "REQ-Ordered - "<< sb_ord_req << "\n";
    sc_assert(0);
  }else{
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp OK   : <<  " << rcv_wr_resp << "\n";
    wr_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of WRITE Response Verify


// 判断两个读响应数据是否相同
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_rd_data (axi4_::ReadPayload &rcv_rd_data, axi4_::ReadPayload &sb_rd_data) {
  unsigned tid_mask = (1 << dnp::ID_W) - 1;
  return ((rcv_rd_data.id & tid_mask) == (sb_rd_data.id & tid_mask) &&
          rcv_rd_data.data  == sb_rd_data.data  &&
          rcv_rd_data.resp  == sb_rd_data.resp  &&
          rcv_rd_data.last  == sb_rd_data.last  //&&
          //rcv_rd_data.ruser == sb_rd_data.ruser
  );
};

// 判断两个写响应是否相同
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_wr_resp (axi4_::WRespPayload &rcv_wr_resp, axi4_::WRespPayload &sb_wr_resp) {
  unsigned tid_mask = (1<<dnp::ID_W)-1;
  return ((rcv_wr_resp.id & tid_mask) == (sb_wr_resp.id & tid_mask) &&
          rcv_wr_resp.resp  == sb_wr_resp.resp  //&&
          //rcv_wr_resp.buser == sb_wr_resp.buser
  );
};

// 地址解析，用于查看addr在哪个slave上
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
unsigned axi_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::mem_map_resolve(axi4_::Addr &addr){
  for (int i = 0; i < SLAVE_NUM; ++i) {
    if (addr >= addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
  }
  NVHLS_ASSERT_MSG(0, "Target Addr not found!");
  return 0; // Or send 404
}


#endif // AXI_MASTER_H
