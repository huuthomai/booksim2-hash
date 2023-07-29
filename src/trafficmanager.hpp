// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _TRAFFICMANAGER_HPP_
#define _TRAFFICMANAGER_HPP_

#include <list>
#include <map>
#include <set>
#include <cassert>

#include "module.hpp"
#include "config_utils.hpp"
#include "network.hpp"
#include "flit.hpp"
#include "buffer_state.hpp"
#include "stats.hpp"
#include "traffic.hpp"
#include "routefunc.hpp"
#include "outputset.hpp"
#include "injection.hpp"

// HANS: Reordering options
// #define PACKET_GRAN_ORDER // Packet granularity reordering
#define MESSAGE_GRAN_ORDER // Message granularity ordering
// #define ORDER_AS_GAP

// THO: Statistic options
// #define CHAN_UTILS
// #define WAIT_TIME


// HANS: Additionals for reordering, based on Merlin's reorderLinkControl
class Compare {
  public:
    bool operator() (Flit* l, Flit* r) const { return l->id > r->id; }
};

struct ReorderInfo {
#ifdef PACKET_GRAN_ORDER
  uint send;
  uint recv;
  priority_queue<Flit*, vector<Flit*>, Compare > q;
#else
  map<int, pair<int, priority_queue<Flit*, vector<Flit*>, Compare > > > q;
#endif

  ReorderInfo()
#ifdef PACKET_GRAN_ORDER
  : send(0),
    recv(0)
#endif
  {}
};

//register the requests to a node
class PacketReplyInfo;

class TrafficManager : public Module {

private:

  vector<vector<int> > _packet_size_rate;
  vector<int> _packet_size_max_val;

protected:

  vector<vector<int> > _packet_size;
  vector<vector<int> > _message_size;
  vector<int> _read_request_message_size;
  vector<int> _read_reply_message_size;
  vector<int> _write_request_message_size;
  vector<int> _write_reply_message_size;
  int _cur_mid;

  int _nodes;
  int _routers;
  int _vcs;

  vector<Network *> _net;
  vector<vector<Router *> > _router;

  // ============ Traffic ============
  int    _classes;

  vector<double> _load;

  vector<int> _use_read_write;
  vector<double> _write_fraction;
  vector<double> _mixed_msg_fraction;

  vector<int> _read_request_size;
  vector<int> _read_reply_size;
  vector<int> _write_request_size;
  vector<int> _write_reply_size;

  int _transient_time;
  vector<string> _traffic;
  vector<string> _traffic_transient;

  vector<int> _class_priority;

  vector<vector<int> > _last_class;

  vector<TrafficPattern *> _traffic_pattern;
  vector<TrafficPattern *> _traffic_pattern_transient;
  vector<InjectionProcess *> _injection_process;

  // ============ Message priorities ============
  enum ePriority { class_based, age_based, network_age_based, local_age_based, queue_length_based, hop_count_based, sequence_based, none };

  ePriority _pri_type;

  // ============ Injection VC states  ============
  vector<vector<BufferState *> > _buf_states;
#ifdef TRACK_FLOWS
  vector<vector<vector<int> > > _outstanding_credits;
  vector<vector<vector<queue<int> > > > _outstanding_classes;
#endif
  vector<vector<vector<int> > > _last_vc;

  // ============ Routing ============
  tRoutingFunction _rf;
  bool _lookahead_routing;
  bool _noq;

  // ============ Injection queues ============
  vector<vector<int> > _qtime;
  vector<vector<bool> > _qdrained;
  vector<vector<list<Flit *> > > _partial_packets;

  vector<map<int, Flit *> > _total_in_flight_flits;
  vector<map<int, Flit *> > _measured_in_flight_flits;
  vector<map<int, Flit *> > _retired_packets;
  bool _empty_network;

  bool _hold_switch_for_packet;

  // ============ physical sub-networks =========
  int _subnets;

  vector<int> _subnet;

  // ============ deadlock ==========
  int _deadlock_timer;
  int _deadlock_warn_timeout;

  // ============ request & replies ==========================
  vector<int> _packet_seq_no;
  vector<int> _message_seq_no;
  vector<list<PacketReplyInfo*> > _repliesPending;
  vector<int> _requestsOutstanding;

  // ============ Statistics ============
  vector<Stats *> _plat_stats;     
  vector<double> _overall_min_plat;  
  vector<double> _overall_avg_plat;  
  vector<double> _overall_max_plat;  

  vector<Stats *> _nlat_stats;     
  vector<double> _overall_min_nlat;  
  vector<double> _overall_avg_nlat;  
  vector<double> _overall_max_nlat;  

  // THO: ROB occupancy
  vector<Stats *> _rob_stats;  
  vector<double> _overall_min_rob;  
  vector<double> _overall_avg_rob;  
  vector<double> _overall_max_rob;   

  vector<Stats *> _flat_stats;     
  vector<double> _overall_min_flat;  
  vector<double> _overall_avg_flat;  
  vector<double> _overall_max_flat;  

  vector<Stats *> _frag_stats;
  vector<double> _overall_min_frag;
  vector<double> _overall_avg_frag;
  vector<double> _overall_max_frag;

  vector<vector<Stats *> > _pair_plat;
  vector<vector<Stats *> > _pair_nlat;
  vector<vector<Stats *> > _pair_flat;

  // HANS: Reordering latency statistics
  vector<Stats *> _rlat_stats;     
  vector<double> _overall_min_rlat;  
  vector<double> _overall_avg_rlat;  
  vector<double> _overall_max_rlat;

  // HANS: Request-reply latency statistics
  vector<Stats *> _read_plat_stats;     
  vector<double> _overall_min_read_plat;  
  vector<double> _overall_avg_read_plat;  
  vector<double> _overall_max_read_plat;

  vector<Stats *> _small_read_plat_stats;     
  vector<double> _overall_min_small_read_plat;  
  vector<double> _overall_avg_small_read_plat;  
  vector<double> _overall_max_small_read_plat;

  vector<Stats *> _write_plat_stats;     
  vector<double> _overall_min_write_plat;  
  vector<double> _overall_avg_write_plat;  
  vector<double> _overall_max_write_plat;

  vector<Stats *> _read_nlat_stats;     
  vector<double> _overall_min_read_nlat;  
  vector<double> _overall_avg_read_nlat;  
  vector<double> _overall_max_read_nlat;

  vector<Stats *> _write_nlat_stats;     
  vector<double> _overall_min_write_nlat;  
  vector<double> _overall_avg_write_nlat;  
  vector<double> _overall_max_write_nlat;

  vector<Stats *> _read_rlat_stats;     
  vector<double> _overall_min_read_rlat;  
  vector<double> _overall_avg_read_rlat;  
  vector<double> _overall_max_read_rlat;

  vector<Stats *> _write_rlat_stats;     
  vector<double> _overall_min_write_rlat;  
  vector<double> _overall_avg_write_rlat;  
  vector<double> _overall_max_write_rlat;

#ifdef WAIT_TIME
  // Endpoint wait time statistics
  vector<Stats *> _ewlat_stats;     
  vector<double> _overall_min_ewlat;  
  vector<double> _overall_avg_ewlat;  
  vector<double> _overall_max_ewlat;

  // Service time statistics
  vector<Stats *> _service_stats;     
  vector<double> _overall_min_service;  
  vector<double> _overall_avg_service;  
  vector<double> _overall_max_service;

  vector<Stats *> _upservice_stats;     
  vector<double> _overall_min_upservice;  
  vector<double> _overall_avg_upservice;  
  vector<double> _overall_max_upservice;

  vector<Stats *> _downservice_stats;     
  vector<double> _overall_min_downservice;  
  vector<double> _overall_avg_downservice;  
  vector<double> _overall_max_downservice;
#endif

#ifdef CHAN_UTILS
  static const int _num_channels = 512; // Uplink + downlink
  vector<vector<Stats *> >  _chanutil_stats;     
  vector<vector<Stats *> >  _chanutil_freq_stats;     
  vector<vector<double> > _overall_min_chanutil;  
  vector<vector<double> > _overall_avg_chanutil;  
  vector<vector<double> > _overall_max_chanutil;   
#endif

  vector<Stats *> _hop_stats;
  vector<double> _overall_hop_stats;

  vector<vector<int> > _sent_packets;
  vector<double> _overall_min_sent_packets;
  vector<double> _overall_avg_sent_packets;
  vector<double> _overall_max_sent_packets;
  vector<vector<int> > _accepted_packets;
  vector<double> _overall_min_accepted_packets;
  vector<double> _overall_avg_accepted_packets;
  vector<double> _overall_max_accepted_packets;
  vector<vector<int> > _sent_flits;
  vector<double> _overall_min_sent;
  vector<double> _overall_avg_sent;
  vector<double> _overall_max_sent;
  vector<vector<int> > _accepted_flits;
  vector<double> _overall_min_accepted;
  vector<double> _overall_avg_accepted;
  vector<double> _overall_max_accepted;

#ifdef TRACK_STALLS
  vector<vector<int> > _buffer_busy_stalls;
  vector<vector<int> > _buffer_conflict_stalls;
  vector<vector<int> > _buffer_full_stalls;
  vector<vector<int> > _buffer_reserved_stalls;
  vector<vector<int> > _crossbar_conflict_stalls;
  vector<double> _overall_buffer_busy_stalls;
  vector<double> _overall_buffer_conflict_stalls;
  vector<double> _overall_buffer_full_stalls;
  vector<double> _overall_buffer_reserved_stalls;
  vector<double> _overall_crossbar_conflict_stalls;
#endif

  vector<int> _slowest_packet;
  vector<int> _slowest_flit;
  vector<int> _longest_service_time;

  map<string, Stats *> _stats;

  // HANS: Additionals for reordering
  int _flit_types = 2;
  vector<vector<vector<ReorderInfo*> > > _reordering_vect;
  vector<vector<int> > _reordering_vect_size;
  int _reordering_vect_maxsize;

  vector<vector<int> > source_resend;
  vector<vector<vector<Flit*> > > _reordering_vect_flit;

  // ============ Simulation parameters ============
  enum eSimState { warming_up, running, draining, done };
  eSimState _sim_state;

  bool _measure_latency;

  int   _reset_time;
  int   _drain_time;

  int   _total_sims;
  int   _sample_period;
  int   _max_samples;
  int   _warmup_periods;

  int   _include_queuing;

  vector<int> _measure_stats;
  bool _pair_stats;

  vector<double> _latency_thres;

  vector<double> _stopping_threshold;
  vector<double> _acc_stopping_threshold;

  vector<double> _warmup_threshold;
  vector<double> _acc_warmup_threshold;

  int _cur_id;
  int _cur_pid;
  int _time;

  // Tho: Local PID at generation for each router
  vector<int> _cur_lpid;

  set<int> _flits_to_watch;
  set<int> _packets_to_watch;

  bool _print_csv_results;

  //flits to watch
  ostream * _stats_out;

#ifdef TRACK_FLOWS
  vector<vector<int> > _injected_flits;
  vector<vector<int> > _ejected_flits;
  ostream * _injected_flits_out;
  ostream * _received_flits_out;
  ostream * _stored_flits_out;
  ostream * _sent_flits_out;
  ostream * _outstanding_credits_out;
  ostream * _ejected_flits_out;
  ostream * _active_packets_out;
#endif

#ifdef TRACK_CREDITS
  ostream * _used_credits_out;
  ostream * _free_credits_out;
  ostream * _max_credits_out;
#endif

  // THO: Node types
  int _num_compute_nodes;
  int _num_memory_nodes;
  // int _num_benign_nodes;

  // ============ Internal methods ============ 
protected:

  virtual void _RetireFlit( Flit *f, int dest );

  virtual void _Inject();
  void _Step( );

  bool _PacketsOutstanding( ) const;
  

  // THO: Message-based simulation
  virtual int  _IssueMessage( int source, int cl );
  virtual void _GenerateMessage( int source, int size, int cl, int time );
  virtual int _GetNextMessageSize(int cl) const;
  virtual double _GetAverageMessageSize(int cl) const;

  virtual void _ClearStats( );

  void _ComputeStats( const vector<int> & stats, int *sum, int *min = NULL, int *max = NULL, int *min_pos = NULL, int *max_pos = NULL ) const;

  virtual bool _SingleSim( );

  void _DisplayRemaining( ostream & os = cout ) const;
  
  void _LoadWatchList(const string & filename);

  virtual void _UpdateOverallStats();

  virtual string _OverallStatsCSV(int c = 0) const;

  int _GetNextPacketSize(int cl) const;
  double _GetAveragePacketSize(int cl) const;

  // HANS: Additionals for reordering
  virtual int FindType(Flit::FlitType type){
    if ((type == Flit::READ_REQUEST) || (type == Flit::WRITE_REQUEST))
      return 0;
    else if ((type == Flit::READ_REPLY) || (type == Flit::WRITE_REPLY))
      return 1;
    else
      return 0;
  }

public:

  static TrafficManager * New(Configuration const & config, 
			      vector<Network *> const & net);

  TrafficManager( const Configuration &config, const vector<Network *> & net );
  virtual ~TrafficManager( );

  bool Run( );

  virtual void WriteStats( ostream & os = cout ) const ;
  virtual void UpdateStats( ) ;
  virtual void DisplayStats( ostream & os = cout ) const ;
  virtual void DisplayOverallStats( ostream & os = cout ) const ;
  virtual void DisplayOverallStatsCSV( ostream & os = cout ) const ;
  virtual void DisplayAvgLatFrequently( ostream & os = cout, int period = 0 ) const ;

  inline int getTime() { return _time;}
  Stats * getStats(const string & name) { return _stats[name]; }

};

template<class T>
ostream & operator<<(ostream & os, const vector<T> & v) {
  for(size_t i = 0; i < v.size() - 1; ++i) {
    os << v[i] << ",";
  }
  os << v[v.size()-1];
  return os;
}

#endif
