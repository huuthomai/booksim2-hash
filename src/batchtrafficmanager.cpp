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

////////////////////////////////////////////////////////////////////////
//
// Modified Batch mode:
//  - Active/Inactive nodes
//  - Message simulation
//  - Injection rate (to use original batch, use load=0.0)
//
////////////////////////////////////////////////////////////////////////
//
//  $Author: Tho Mai - CSNL KAIST $
//  $Date: Apr 2023 $
// 
////////////////////////////////////////////////////////////////////////

#include <limits>
#include <sstream>
#include <fstream>

#include "packet_reply_info.hpp"
#include "random_utils.hpp"
#include "batchtrafficmanager.hpp"

BatchTrafficManager::BatchTrafficManager( const Configuration &config, 
					  const vector<Network *> & net )
: TrafficManager(config, net), _last_id(-1), _last_pid(-1), 
   _overall_min_batch_time(0), _overall_avg_batch_time(0), 
   _overall_max_batch_time(0)
{

  cout << "Init batch mode ..." << endl;

  _max_outstanding = config.GetInt ("max_outstanding_requests");  

  _batch_size = config.GetInt( "batch_size" );
  _batch_count = config.GetInt( "batch_count" );

  _batch_time = new Stats( this, "batch_time", 1.0, 1000 );
  _stats["batch_time"] = _batch_time;
  
  string sent_packets_out_file = config.GetStr( "sent_packets_out" );
  if (sent_packets_out_file == "") {
    _sent_packets_out = NULL;
  } else {
    _sent_packets_out = new ofstream(sent_packets_out_file.c_str());
  }


  _load = config.GetFloatArray("batch_injection_rate"); 
  if (_load.empty()) {
      _load.push_back(config.GetFloat("batch_injection_rate"));
  } 
  _load.resize(_classes, _load.back());
  batch_send_all.resize(_classes, false);


  _read_request_message_size = config.GetIntArray("read_request_message_size");
  if (_read_request_message_size.empty()) {
      _read_request_message_size.push_back(config.GetInt("read_request_message_size"));
  }
  _read_request_message_size.resize(_classes, _read_request_message_size.back());

  _read_reply_message_size = config.GetIntArray("read_reply_message_size");
  if (_read_reply_message_size.empty()) {
      _read_reply_message_size.push_back(config.GetInt("read_reply_message_size"));
  }
  _read_reply_message_size.resize(_classes, _read_reply_message_size.back());

  _write_request_message_size = config.GetIntArray("write_request_message_size");
  if (_write_request_message_size.empty()) {
      _write_request_message_size.push_back(config.GetInt("write_request_message_size"));
  }
  _write_request_message_size.resize(_classes, _write_request_message_size.back());

  _write_reply_message_size = config.GetIntArray("write_reply_message_size");
  if (_write_reply_message_size.empty()) {
      _write_reply_message_size.push_back(config.GetInt("write_reply_message_size"));
  }
  _write_reply_message_size.resize(_classes, _write_reply_message_size.back());


  string message_size_str = config.GetStr("message_size");
  if (message_size_str.empty()) {
      _message_size.push_back(vector<int>(1, config.GetInt("message_size")));
  } else {
      vector<string> message_size_strings = tokenize_str(message_size_str);
      for(size_t i = 0; i < message_size_strings.size(); ++i) {
          _message_size.push_back(tokenize_int(message_size_strings[i]));
      }
  }
  _message_size.resize(_classes, _message_size.back());


  for(int c = 0; c < _classes; ++c) {
      if (_use_read_write[c]) {
          _message_size[c] = 
              vector<int>(1, (_read_request_message_size[c] * _read_request_size[c] + _read_reply_message_size[c] * _read_reply_size[c] +
                              _write_request_message_size[c] * _write_request_size[c] + _write_reply_message_size[c] * _write_reply_size[c]) / 2);
      }
  }

  if (config.GetInt("injection_rate_uses_flits")) {
    for(int c = 0; c < _classes; ++c){
      if (_load[c] == 0) {
        batch_send_all[c] = true;
        cout << "BATCH_SEND_ALL" << endl;
      }
      cout << "LOAD TEST: " << _load[c] << ", MESSAGE SIZE: " << _GetAverageMessageSize(c) << endl;
      _load[c] /= _GetAverageMessageSize(c);
    }
  }

  vector<string> injection_process = config.GetStrArray("injection_process");
  injection_process.resize(_classes, injection_process.back());

  for(int c = 0; c < _classes; ++c){
    cout << "Register load: " << _load[c] << endl;
    _injection_process[c] = InjectionProcess::New(injection_process[c], _nodes, _load[c], &config);
  }


  _cur_mid = 0;


  _message_seq_no.resize(_nodes);

}

BatchTrafficManager::~BatchTrafficManager( )
{
  delete _batch_time;
  if (_sent_packets_out) delete _sent_packets_out;
}

// HANS: Inherit _RetireFlit to enable reordering buffer
void BatchTrafficManager::_RetireFlit(Flit *f, int dest)
{
  // Insert to reordering buffer
  // Push flit to its respective reordering queue

  if (f->watch)
    cout << GetSimTime() << " - Push flit " << f->id << " packet " << f->pid << " message " << f->mid << " to the reordering buffer." << endl;

  // if (f->head){
  if (f->tail){
    // assert(f->dest == dest);
    f->rtime = GetSimTime();
  }

  int type = FindType(f->type);
#ifdef PACKET_GRAN_ORDER
  _reordering_vect[f->src][dest][type]->q.push(f);
#elif defined(MESSAGE_GRAN_ORDER) || defined(ORDER_AS_GAP)
  auto search = _reordering_vect[f->src][dest][type]->q.find(f->mid);

  if (search == _reordering_vect[f->src][dest][type]->q.end()){ // Entry does not exists
    priority_queue<Flit*, vector<Flit*>, Compare > temp;

    auto temp_pair = make_pair(f->mid, make_pair(0, temp));

    assert(_reordering_vect[f->src][dest][type]->q.insert(temp_pair).second);
    search = _reordering_vect[f->src][dest][type]->q.find(f->mid);
  }

  search->second.second.push(f);
#endif

  // THO: record ROB occupancy
  if ((f->src / gC) != (f->dest / gC)) {
    _rob_stats[f->cl]->AddSample( _reordering_vect_size[f->src][dest] );
  }

  // Inject a flit into ROB
  _reordering_vect_size[f->src][dest] += 1;

  // THO: Total ROB
  // int total_rob = 0;
  // for (int s = 0; s < _nodes; s++) {
  //   total_rob += _reordering_vect_size[s][dest];
  // }
  // _rob_stats[f->cl]->AddSample( total_rob );

  assert(_reordering_vect_size[f->src][dest] >= 0);

  if (_reordering_vect_size[f->src][dest] > _reordering_vect_maxsize){
    _reordering_vect_maxsize = _reordering_vect_size[f->src][dest];
  }

#ifdef PACKET_GRAN_ORDER
  while ((!_reordering_vect[f->src][dest][type]->q.empty()) && (_reordering_vect[f->src][dest][type]->q.top()->packet_seq <= _reordering_vect[f->src][dest][type]->recv)){
    Flit* temp = _reordering_vect[f->src][dest][type]->q.top();

    if (temp->tail)
      _reordering_vect[f->src][dest][type]->recv += 1;
#elif defined(MESSAGE_GRAN_ORDER)
  bool is_delete = false;

  while ((!search->second.second.empty()) && (search->second.second.top()->packet_seq <= (unsigned)search->second.first)){
    Flit* temp = search->second.second.top();

    if (temp->tail)
      search->second.first += 1;
#elif defined(ORDER_AS_GAP)
  bool is_delete = false;

  
#endif

    _last_id = temp->id;
    _last_pid = temp->pid;


    // Eject a flit from ROB
    _reordering_vect_size[temp->src][dest] -= 1;
    assert(_reordering_vect_size[temp->src][dest] >= 0);

    TrafficManager::_RetireFlit(temp, dest);

#ifdef PACKET_GRAN_ORDER
    _reordering_vect[f->src][dest][type]->q.pop();
#else
    search->second.second.pop();

    if ((temp->msg_tail) && (temp->tail)){
      assert(search->second.second.empty());
      is_delete = true;
    }
#endif
  }

#ifndef PACKET_GRAN_ORDER
  if (is_delete){
    _reordering_vect[f->src][dest][type]->q.erase(search);
  }
#endif

  // HANS: Without reordering buffer
  // _last_id = f->id;
  // _last_pid = f->pid;
  // TrafficManager::_RetireFlit(f, dest);
}


int BatchTrafficManager::_IssueMessage( int source, int cl )
{
  int result = 0;
  if (_use_read_write[cl]) { //read write packets
    //check queue for waiting replies.
    //check to make sure it is on time yet
    if (!_repliesPending[source].empty()) {
      if (_repliesPending[source].front()->time <= _time) {
	      result = -1;
      }
    } else {
      // THO: Active and hotspot
      if ((_compute_nodes.count(source) > 0) && !_hs_dests.count(source)) {
      // if (1){
        if ((_injection_process[cl]->test(source) || batch_send_all[cl]) && (_message_seq_no[source] < _batch_size) && ((_max_outstanding <= 0) || (_requestsOutstanding[source] < _max_outstanding))) {
	        //coin toss to determine request type.
	        result = (RandomFloat() < _write_fraction[cl]) ? 2 : 1;

	        _requestsOutstanding[source]++;
        }
      }
    }
  } else { //normal
    // THO: Active and hotspot
    if ((_compute_nodes.count(source) > 0) && !_hs_dests.count(source)) {
    // if (1){
      if ((_injection_process[cl]->test(source) || batch_send_all[cl]) && (_message_seq_no[source] < _batch_size) && ((_max_outstanding <= 0) || (_requestsOutstanding[source] < _max_outstanding))) {
        result = _GetNextMessageSize(cl);
        _requestsOutstanding[source]++;
      }
    }
  }
  // if (result != 0) {
  if (result > 0) {
    _message_seq_no[source]++;
  }
  return result;
}


void BatchTrafficManager::_ClearStats( )
{
  TrafficManager::_ClearStats();
  _batch_time->Clear( );
}

bool BatchTrafficManager::_SingleSim( )
{
  int batch_index = 0;
  while(batch_index < _batch_count) {
    _message_seq_no.assign(_nodes, 0);
    _last_id = -1;
    _last_pid = -1;
    _sim_state = running;
    int start_time = _time;
    bool batch_complete;
    cout << "Sending batch " << batch_index + 1 << " (" << _batch_size << " packets)..." << endl;
    do {
      _Step();
      batch_complete = true;
      for(int i = 0; i < _nodes; ++i) {
        // THO: Active and hotspot
        if ((_compute_nodes.count(i) > 0) && !_hs_dests.count(i)) {
        // if (1){  
          if (_message_seq_no[i] < _batch_size) {
	          batch_complete = false;
	          break;
          }
	      }
      }
      // if (_sent_packets_out) {
	      // *_sent_packets_out << _packet_seq_no << endl;
      // }
    } while(!batch_complete);
    cout << "Batch injected. Time used is " << _time - start_time << " cycles." << endl;

    int sent_time = _time;
    cout << "Waiting for batch to complete..." << endl;

    int empty_steps = 0;
    
    bool packets_left = false;
    for(int c = 0; c < _classes; ++c) {
      packets_left |= !_total_in_flight_flits[c].empty();
    }
    
    while( packets_left ) { 
      _Step( ); 
      
      ++empty_steps;
      
      if ( empty_steps % 1000 == 0 ) {
        _DisplayRemaining( ); 
        cout << ".";
      }
      
      packets_left = false;
      for(int c = 0; c < _classes; ++c) {
        packets_left |= !_total_in_flight_flits[c].empty();
      }
    }
    cout << endl;
    cout << "Batch received. Time used is " << _time - sent_time << " cycles." << endl
	       << "Last packet was " << _last_pid << ", last flit was " << _last_id << "." << endl;

    _batch_time->AddSample(_time - start_time);

    cout << _sim_state << endl;

    UpdateStats();
    DisplayStats();
        
    ++batch_index;
  }
  _sim_state = draining;
  _drain_time = _time;
  return 1;
}

void BatchTrafficManager::_UpdateOverallStats() {
  TrafficManager::_UpdateOverallStats();
  _overall_min_batch_time += _batch_time->Min();
  _overall_avg_batch_time += _batch_time->Average();
  _overall_max_batch_time += _batch_time->Max();
}
  
string BatchTrafficManager::_OverallStatsCSV(int c) const
{
  ostringstream os;
  os << TrafficManager::_OverallStatsCSV(c) << ','
     << _overall_min_batch_time / (double)_total_sims << ','
     << _overall_avg_batch_time / (double)_total_sims << ','
     << _overall_max_batch_time / (double)_total_sims;
  return os.str();
}

void BatchTrafficManager::WriteStats(ostream & os) const
{
  TrafficManager::WriteStats(os);
  os << "batch_time = " << _batch_time->Average() << ";" << endl;
}    

void BatchTrafficManager::DisplayStats(ostream & os) const {
  TrafficManager::DisplayStats();
  os << "Minimum batch duration = " << _batch_time->Min() << endl;
  os << "Average batch duration = " << _batch_time->Average() << endl;
  os << "Maximum batch duration = " << _batch_time->Max() << endl;
}

void BatchTrafficManager::DisplayOverallStats(ostream & os) const {
  TrafficManager::DisplayOverallStats(os);
  os << "Overall min batch duration = " << _overall_min_batch_time / (double)_total_sims
     << " (" << _total_sims << " samples)" << endl
     << "Overall avg batch duration = " << _overall_avg_batch_time / (double)_total_sims
     << " (" << _total_sims << " samples)" << endl
     << "Overall max batch duration = " << _overall_max_batch_time / (double)_total_sims
     << " (" << _total_sims << " samples)" << endl;
}
