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

/*routefunc.cpp
 *
 *This is where most of the routing functions reside. Some of the topologies
 *has their own "register routing functions" which must be called to access
 *those routing functions. 
 *
 *After writing a routing function, don't forget to register it. The reg 
 *format is rfname_topologyname. 
 *
 */

#include <map>
#include <cstdlib>
#include <cassert>
#include <math.h>
#include <cmath>
#include <numeric>
#include <bits/stdc++.h>
#include <algorithm>
#include <stdlib.h>     /* abs */

#include "booksim.hpp"
#include "routefunc.hpp"
#include "kncube.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "fattree.hpp"
#include "fattree2.hpp"
#include "tree4.hpp"
#include "qtree.hpp"
#include "cmesh.hpp"



map<string, tRoutingFunction> gRoutingFunctionMap;

/* Global information used by routing functions */

int gNumVCs;

/* Add more functions here
 *
 */

// ============================================================
//  Balfour-Schultz
int gReadReqBeginVC, gReadReqEndVC;
int gWriteReqBeginVC, gWriteReqEndVC;
int gReadReplyBeginVC, gReadReplyEndVC;
int gWriteReplyBeginVC, gWriteReplyEndVC;

// ============================================================
//  QTree: Nearest Common Ancestor
// ===
void qtree_nca( const Router *r, const Flit *f,
		int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int height = QTree::HeightFromID( r->GetID() );
    int pos    = QTree::PosFromID( r->GetID() );
    
    int dest   = f->dest;
    
    for (int i = height+1; i < gN; i++) 
      dest /= gK;
    if ( pos == dest / gK ) 
      // Route down to child
      out_port = dest % gK ; 
    else
      // Route up to parent
      out_port = gK;        

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

// ============================================================
//  Tree4: Nearest Common Ancestor w/ Adaptive Routing Up
// ===
void tree4_anca( const Router *r, const Flit *f,
		 int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int range = 1;
  
  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int dest = f->dest;
    
    const int NPOS = 16;
    
    int rH = r->GetID( ) / NPOS;
    int rP = r->GetID( ) % NPOS;
    
    if ( rH == 0 ) {
      dest /= 16;
      out_port = 2 * dest + RandomInt(1);
    } else if ( rH == 1 ) {
      dest /= 4;
      if ( dest / 4 == rP / 2 )
	out_port = dest % 4;
      else {
	out_port = gK;
	range = gK;
      }
    } else {
      if ( dest/4 == rP )
	out_port = dest % 4;
      else {
	out_port = gK;
	range = 2;
      }
    }
    
    //  cout << "Router("<<rH<<","<<rP<<"): id= " << f->id << " dest= " << f->dest << " out_port = "
    //       << out_port << endl;

  }

  outputs->Clear( );

  for (int i = 0; i < range; ++i) 
    outputs->AddRange( out_port + i, vcBegin, vcEnd );
}

// ============================================================
//  Tree4: Nearest Common Ancestor w/ Random Routing Up
// ===
void tree4_nca( const Router *r, const Flit *f,
		int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int dest = f->dest;
    
    const int NPOS = 16;
    
    int rH = r->GetID( ) / NPOS;
    int rP = r->GetID( ) % NPOS;
    
    if ( rH == 0 ) {
      dest /= 16;
      out_port = 2 * dest + RandomInt(1);
    } else if ( rH == 1 ) {
      dest /= 4;
      if ( dest / 4 == rP / 2 )
        out_port = dest % 4;
      else
        out_port = gK + RandomInt(gK-1);
    } else {
      if ( dest/4 == rP )
        out_port = dest % 4;
      else
        out_port = gK + RandomInt(1);
    }
    
    //  cout << "Router("<<rH<<","<<rP<<"): id= " << f->id << " dest= " << f->dest << " out_port = "
    //       << out_port << endl;

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

// ============================================================
//  FATTREE2: Nearest Common Ancestor w/ Random  Routing Up
// ===
void fattree2_nca( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    // For 2 level only
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int router_depth = (router_id < (gK/2)) ? 0 : 1; //which level
    

    //NCA reached going down
    if (router_depth == 0) {
      // out_port = 2*(dest/gK) + RandomInt(1);
      out_port = 2*(dest/gK) + (in_channel % 2);
    }
    else {
      //routing up
      if ((f->hops == 0) && ((f->src / gK) != (dest / gK))) {
        assert(in_channel<gK); //came from a up channel
        out_port = gK+RandomInt(gK-1);
      }
      //ejection
      else {
        out_port = dest%gK;
      }
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree2_nca_hybrid( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    // For 2 level only
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int router_depth = (router_id < (gK/2)) ? 0 : 1; //which level
    

    //NCA reached going down
    if (router_depth == 0) {
      out_port = 2*(dest/gK) + (in_channel % 2);
    }
    else {
      //routing up
      if ((f->hops == 0) && ((f->src / gK) != (dest / gK))) {
        assert(in_channel<gK); //came from a up channel
        // int port1 = gK + (in_channel % gC);   // Source hashing port
        int port1 = gK + (f->dest % gC);  // Destination hashing 
        // int port1 = gK + RandomInt(gK-1);  // Random port
        // int port2 = gK + RandomInt(gK-1);  // Random port

        // int key = (gK * (r->offset / r->active_node_total))+ f->src;
        // int key = (gK * (r->offset / gK)) + f->src;
        // int key = (gK * (r->offset / gK)) + r->pattern_init[f->src % gK];

        int key = r->random_offset + (r->GetID() - gK)*gK + ((in_channel + r->random_src_offset) % gK);

        unsigned long fibonacci_mult = 11400714819323198485;  // uint64_t gives same results
        int port2 = gK + ((unsigned long)(key * fibonacci_mult) >> 60);

        r->inc_hash++;
        

        // Switching scheme #1: 1-way threshold
        int threshold = 0;
        if ((r->GetUsedCredit(port1) + r->flit_count[port1]) > (r->GetUsedCredit(port2) + r->flit_count[port2] + threshold)) {
          out_port = port2;
        } else {
          out_port = port1;
        }
        out_port = port1;
        // out_port = port2;
      }
      //ejection
      else {
        out_port = dest%gK;
      }
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}


// ============================================================
//  FATTREE: Nearest Common Ancestor w/ Random  Routing Up
// ===
void fattree_nca( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel
      out_port = gK + RandomInt(gK-1);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_sourcehash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel
      out_port = gK + (in_channel % gC);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree_nca_desthash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel
      out_port = gK + (f->dest % gC);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_xorhash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel

      // if (f->dest > f->src) {
      //   out_port = gK + ((f->dest - f->src) % gK);
      // } else {
      //   assert((f->src / gC) != (f->dest / gC));
      //   out_port = gK + ((f->src % gK) ^ (f->dest % gK));
      // }

      out_port = gK + ((f->src % gK) ^ (f->dest / gK)); // YARC?
      // out_port = gK + (((f->src % gK) ^ f->dest) % gK); // YARC?
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree_nca_roundrobin( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
	      out_port = dest%gK;
      } else {	
	      //find the down port for the destination
	      int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
	      out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel

      // // Roundrobin with different pattern
      // out_port = gK + r->GenerateCyclic(gK, r->src_avai);

      // Roundrobin with the same pattern
      out_port = gK + r->GetRROffset();
      r->IncrementRROffset(gK);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_fibonaccihash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
	      out_port = dest%gK;
      } else {	
	      //find the down port for the destination
	      int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
	      out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel

      // int key = f->src * (gK * gC) + f->dest;
      // int key = ((gC * gK) * (gC * gK) * r->GetRROffset()) + (f->src * (gC * gK) + f->dest);
      // int key = gC * (gK * gC) + r->GetRROffset();
      // int key = f->dest * (gK * gC) + r->GetRROffset();
      int key = r->inc_hash;
      unsigned long fibonacci_mult = 11400714819323198485; // (2 ^ (64)) / 1.61803398875, 64 is the word size

      out_port = gK + ((unsigned long)(key * fibonacci_mult) >> 60);
      r->inc_hash++;
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree_nca_fibonacci_pid_hash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
	      out_port = dest%gK;
      } else {	
	      //find the down port for the destination
	      int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
	      out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel

      int key = f->src * (gK * gK) + f->dest + f->pid;
      unsigned long fibonnaci_mult = 11400714819323198485; // (2 ^ (64)) / 1.61803398875, 64 is the word size

      out_port = gK + ((unsigned long)(key * fibonnaci_mult) >> 60);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree_nca_fibonacci_inj_hash( const Router *r, const Flit *f,
               int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
	      out_port = dest%gK;
      } else {	
	      //find the down port for the destination
	      int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
	      out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel

      // int key = ((gC * gK) * (gC * gK) * r->GetInjectedPacket(f->src % gC)) + (f->src * (gC * gK) + f->dest);
      // int key = (gC * r->GetInjectedPacket(f->src % gC)) + (f->src * gC + f->dest);
      int key = r->GetInjectedPacket(f->src % gC) ^ (f->src ^ f->dest);
      unsigned long fibonacci_mult = 11400714819323198485; // (2 ^ (64)) / 1.61803398875, 64 is the word size

      out_port = gK + ((unsigned long)(key * fibonacci_mult) >> 60);
    }
  }  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}


// ============================================================
//  FATTREE: Nearest Common Ancestor w/ Adaptive Routing Up
// ===
void fattree_anca( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel<gK);//came from a up channel
      out_port = gK;
      int random1 = RandomInt(gK-1); // Chose two ports out of the possible at random, compare loads, choose one.
      int random2 = RandomInt(gK-1);
      // if ((r->GetUsedCredit(out_port + random1) + r->flit_count[out_port + random1]) > (r->GetUsedCredit(out_port + random2) + r->flit_count[out_port + random2])) {
      if (r->GetUsedCredit(out_port + random1) > r->GetUsedCredit(out_port + random2)) {
        out_port = out_port + random2;
      } else {
        out_port =  out_port + random1;
      }
      // if ((out_port-gK == 7) && (r->GetID() == 40))
      //   cout << GetSimTime() << "\t(Route) Credit: " << r->GetUsedCredit(7) << endl;
    }
    // r->flit_count[out_port] += f->packet_size;
  }  
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}

void fattree_anca_greedy( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));
  int out_port;
  if(inject) {
    out_port = -1;
  } else {
    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage &&
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first
      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1));
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK);//came from a up channel
      int min_occupancy = r->GetUsedCredit(gK);
      // int min_idx = 0;
      for (int iter = 1; iter < gK; iter++){
        if (r->GetUsedCredit(gK + iter) < min_occupancy){
          min_occupancy = r->GetUsedCredit(gK + iter);
          // min_idx = iter;
        }
      }
      // Choose randomly for tie breaker
      vector<int> temp;
      for (int iter = 0; iter < gK; iter++){
        if (r->GetUsedCredit(gK + iter) == min_occupancy){
          temp.push_back(iter);
        }
      }
      int random = RandomInt(temp.size() - 1);
      out_port = gK + temp[random];
    }
  }
  outputs->Clear( );
  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_hybrid( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
        if (in_channel < gK) {
          r->inc_hash++;
        }
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK); //came from a up channel

      // int port1 = gK + (in_channel % gC);   // Source hashing port
      int port1 = gK + (f->dest % gC);  // Destination hashing 
      // int port1 = gK + (int(f->dest / pow(log2(gK),2*(gN-router_depth-1))) % gC); // Only work with k >= 4
      // int port1 = gK + RandomInt(gK-1);  // Random port
      // int port2 = gK + RandomInt(gK-1);  // Random port


      // int key = (gK * (r->offset / r->active_node_total)) + f->src;
      // int key = (gK * (r->offset / gK)) + f->src;
      // int key = (gK * (r->offset / gK)) + (r->GetID() - gK)*gK + in_channel;
      int key = r->random_offset + (r->GetID() - gK)*gK + ((in_channel + r->random_src_offset) % gK);


      unsigned long fibonacci_mult = 11400714819323198485;  // uint64_t gives same results
      int port2 = gK + ((unsigned long)(key * fibonacci_mult) >> 60);

      r->inc_hash++;
      
      int threshold = 0;

      // Switching scheme #1: 1-way threshold
      if ((r->GetUsedCredit(port1) + r->flit_count[port1]) > (r->GetUsedCredit(port2) + r->flit_count[port2] + threshold)) {
        out_port = port2;
      } else {
        out_port = port1;
      }


      // Switching scheme #2: 2-way threshold
      // if (!r->switch_flag[f->src % gK]) {
      //   if (r->GetUsedCredit(port1) + r->flit_count[port1] > (r->GetUsedCredit(port2) + r->flit_count[port2] + threshold)) {
      //     out_port = port2;
      //     r->switch_flag[f->src % gK] = true;
      //   } else {
      //     out_port = port1;
      //     r->switch_flag[f->src % gK] = false;
      //   }
      // } else {
      //   if (r->GetUsedCredit(port2) + r->flit_count[port2] > (r->GetUsedCredit(port1) + r->flit_count[port1] + threshold)) {
      //     out_port = port1;
      //     r->switch_flag[f->src % gK] = false;
      //   } else {
      //     out_port = port2;
      //     r->switch_flag[f->src % gK] = true;
      //   }  
      // }
    }
  }
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_hybrid_mem( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
        if (in_channel < gK) {
          r->inc_hash++;
        }
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      assert(in_channel < gK); //came from a up channel


      // int port1 = gK + (in_channel % gC);   // Source hashing port
      int port1 = gK + (int(f->dest / pow(log2(gK),2*(gN-router_depth-1))) % gC); // Dest hashing: Only work with k >= 4
      // int port1 = gK + RandomInt(gK-1);  // Random port
      // int port2 = gK + RandomInt(gK-1);  // Random port

      // int key = ((r->offset / gK)) + f->src;
      // int key = (gK * r->offset) + f->src;
      int key = (gK * (r->offset / gK)) + f->src;
      // int key = (gK * (r->offset / gK)) + r->pattern_init[f->src % gK];
      // int key = (gK * (r->offset / gK)) + gK*r->GetID() + in_channel;
      // int key = (gK * (GetSimTime() / gK)) + f->src;
      // int key = ((r->offset / gK)) + f->src;
      unsigned long fibonacci_mult = 11400714819323198485;
      int port2 = gK + ((unsigned long)(key * fibonacci_mult) >> 60);

      int port3 = gK + r->latest_port[in_channel % gK];

      r->inc_hash++;
      
      int threshold = 0;

      // Switching scheme #1: 1-way threshold
      if ((r->GetUsedCredit(port1) + r->flit_count[port1]) > (r->GetUsedCredit(port3) + r->flit_count[port3] + threshold)) {
        out_port = port3;
      } else {
        out_port = port1;
      }
      if ((r->GetUsedCredit(out_port) + r->flit_count[out_port]) > (r->GetUsedCredit(port2) + r->flit_count[port2] + threshold)) {
        out_port = port2;
      } else {
        out_port = out_port;
      }
      r->latest_port[in_channel % gK] = out_port - gK;
    }
    // r->flit_count[out_port] += f->packet_size;
    // r->committed_packet.push_back(make_pair(out_port, f->packet_size));
  }
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_murmurhash3( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
        if (in_channel < gK)
          r->inc_hash++;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK); //came from a up channel

      // // 32-bit (https://gist.github.com/zeux/25b490b07b4873efc08bd37c843777a4)
      // int key = (gK * (r->offset / gK)) + f->src;
      // key ^= key >> 16;
      // key *= 0x85ebca6bu;
      // key ^= key >> 13;
      // key *= 0xc2b2ae35u;
      // key ^= key >> 16;

      // murmur2 - 32-bit (UR plot)
      int key = (gK * (r->offset / gK)) + f->src;
      key ^= key >> 13;
      key *= 0x5bd1e995;
      key ^= key >> 15;

      // // 64-bit (http://zimbry.blogspot.com/2011/09/better-bit-mixing-improving-on.html)
      // long long int key = (gK * (r->offset / gK)) + f->src;
      // key ^= (key >> 33);
      // key *= 0xff51afd7ed558ccd;
      // key ^= (key >> 33);
      // key *= 0xc4ceb9fe1a85ec53;
      // key ^= (key >> 33);


      r->inc_hash++;

      out_port = gK + (key % gK);
    }
  }
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_crc( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
        if (in_channel < gK)
          r->inc_hash++;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK); //came from a up channel

      // int key = (gK * (r->offset / gK)) + f->src;
      int key = r->random_offset + (r->GetID() - gK)*gK + ((in_channel + r->random_src_offset) % gK);

      // char input_chars[sizeof(key)];
      // std::memcpy(input_chars, &key, sizeof(key)); //copy the binary representation of the input integer to the character array
      // out_port = gK + (abs(int(r->CRC8(input_chars,sizeof(input_chars)))) % gK);

      out_port = gK + (abs(int(r->crc16_int(key))) % gK);

      r->inc_hash++;
    }
  }
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}



void fattree_nca_fibonacci( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK); //came from a up channel

      int key = (gK * (r->offset / gK)) + f->src;
      unsigned long fibonacci_mult = 11400714819323198485;
      out_port = gK + ((unsigned long)(key * fibonacci_mult) >> 60);
      r->inc_hash++;
    }
  }
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}


void fattree_nca_portlimit( const Router *r, const Flit *f,
                int in_channel, OutputSet* outputs, bool inject)
{

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));


  int out_port;

  if(inject) {

    out_port = -1;

  } else {


    int dest = f->dest;
    int router_id = r->GetID(); //routers are numbered with smallest at the top level
    int routers_per_level = powi(gK, gN-1);
    int pos = router_id%routers_per_level;
    int router_depth  = router_id/ routers_per_level; //which level
    int routers_per_neighborhood = powi(gK,gN-router_depth-1);
    int router_neighborhood = pos/routers_per_neighborhood; //coverage of this tree
    int router_coverage = powi(gK, gN-router_depth);  //span of the tree from this router
    

    //NCA reached going down
    if(dest <(router_neighborhood+1)* router_coverage && 
       dest >=router_neighborhood* router_coverage){
      //down ports are numbered first

      //ejection
      if(router_depth == gN-1){
        out_port = dest%gK;
      } else {	
        //find the down port for the destination
        int router_branch_coverage = powi(gK, gN-(router_depth+1)); 
        out_port = (dest-router_neighborhood* router_coverage)/router_branch_coverage;
      }
    } else {
      //up ports are numbered last
      assert(in_channel < gK); //came from a up channel

      int port_option = 2;
      int rand_port = RandomInt(gK-1);
      int in_port = f->src % gK;  // Per port
      // int in_port = f->mid % gK;  // Per message
      int port_min = (in_port*port_option) % gK; 
      int port_max = ((in_port+1)*port_option - 1) % gK;


      // Random among "port_option" ports
      if (port_max >= port_min) {
        while (!((rand_port >= port_min) && (rand_port <= port_max))) {
          rand_port = RandomInt(gK-1);
        }
      }
      else {
        while (!((rand_port >= port_min) || (rand_port <= port_max))) {
          rand_port = RandomInt(gK-1);
        }
      }

      // Round-robin among "port_option" ports
      // rand_port = port_min + r->spread_pid[port_min];
      // r->spread_pid[port_min] = (r->spread_pid[port_min] + 1) % port_option;

      out_port = gK + rand_port;
    }
  }  
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}



// ============================================================
//  Mesh - adatpive XY,YX Routing 
//         pick xy or yx min routing adaptively at the source router
// ===

int dor_next_mesh( int cur, int dest, bool descending = false );

void adaptive_xy_yx_mesh( const Router *r, const Flit *f, 
		 int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else if(r->GetID() == f->dest) {

    // at destination router, we don't need to separate VCs by dim order
    out_port = 2*gN;

  } else {

    //each class must have at least 2 vcs assigned or else xy_yx will deadlock
    int const available_vcs = (vcEnd - vcBegin + 1) / 2;
    assert(available_vcs > 0);
    
    int out_port_xy = dor_next_mesh( r->GetID(), f->dest, false );
    int out_port_yx = dor_next_mesh( r->GetID(), f->dest, true );

    // Route order (XY or YX) determined when packet is injected
    //  into the network, adaptively
    bool x_then_y;
    if(in_channel < 2*gN){
      x_then_y =  (f->vc < (vcBegin + available_vcs));
    } else {
      int credit_xy = r->GetUsedCredit(out_port_xy);
      int credit_yx = r->GetUsedCredit(out_port_yx);
      if(credit_xy > credit_yx) {
	x_then_y = false;
      } else if(credit_xy < credit_yx) {
	x_then_y = true;
      } else {
	x_then_y = (RandomInt(1) > 0);
      }
    }
    
    if(x_then_y) {
      out_port = out_port_xy;
      vcEnd -= available_vcs;
    } else {
      out_port = out_port_yx;
      vcBegin += available_vcs;
    }

  }

  outputs->Clear();

  outputs->AddRange( out_port , vcBegin, vcEnd );
  
}

void xy_yx_mesh( const Router *r, const Flit *f, 
		 int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else if(r->GetID() == f->dest) {

    // at destination router, we don't need to separate VCs by dim order
    out_port = 2*gN;

  } else {

    //each class must have at least 2 vcs assigned or else xy_yx will deadlock
    int const available_vcs = (vcEnd - vcBegin + 1) / 2;
    assert(available_vcs > 0);

    // Route order (XY or YX) determined when packet is injected
    //  into the network
    bool x_then_y = ((in_channel < 2*gN) ?
		     (f->vc < (vcBegin + available_vcs)) :
		     (RandomInt(1) > 0));

    if(x_then_y) {
      out_port = dor_next_mesh( r->GetID(), f->dest, false );
      vcEnd -= available_vcs;
    } else {
      out_port = dor_next_mesh( r->GetID(), f->dest, true );
      vcBegin += available_vcs;
    }

  }

  outputs->Clear();

  outputs->AddRange( out_port , vcBegin, vcEnd );
  
}

//
// End Balfour-Schultz
//=============================================================

//=============================================================

int dor_next_mesh( int cur, int dest, bool descending )
{
  if ( cur == dest ) {
    return 2*gN;  // Eject
  }

  int dim_left;

  if(descending) {
    for ( dim_left = ( gN - 1 ); dim_left > 0; --dim_left ) {
      if ( ( cur * gK / gNodes ) != ( dest * gK / gNodes ) ) { break; }
      cur = (cur * gK) % gNodes; dest = (dest * gK) % gNodes;
    }
    cur = (cur * gK) / gNodes;
    dest = (dest * gK) / gNodes;
  } else {
    for ( dim_left = 0; dim_left < ( gN - 1 ); ++dim_left ) {
      if ( ( cur % gK ) != ( dest % gK ) ) { break; }
      cur /= gK; dest /= gK;
    }
    cur %= gK;
    dest %= gK;
  }

  if ( cur < dest ) {
    return 2*dim_left;     // Right
  } else {
    return 2*dim_left + 1; // Left
  }
}

//=============================================================

void dor_next_torus( int cur, int dest, int in_port,
		     int *out_port, int *partition,
		     bool balance = false )
{
  int dim_left;
  int dir;
  int dist2;

  for ( dim_left = 0; dim_left < gN; ++dim_left ) {
    if ( ( cur % gK ) != ( dest % gK ) ) { break; }
    cur /= gK; dest /= gK;
  }
  
  if ( dim_left < gN ) {

    if ( (in_port/2) != dim_left ) {
      // Turning into a new dimension

      cur %= gK; dest %= gK;
      dist2 = gK - 2 * ( ( dest - cur + gK ) % gK );
      
      if ( ( dist2 > 0 ) || 
	   ( ( dist2 == 0 ) && ( RandomInt( 1 ) ) ) ) {
	*out_port = 2*dim_left;     // Right
	dir = 0;
      } else {
	*out_port = 2*dim_left + 1; // Left
	dir = 1;
      }
      
      if ( partition ) {
	if ( balance ) {
	  // Cray's "Partition" allocation
	  // Two datelines: one between k-1 and 0 which forces VC 1
	  //                another between ((k-1)/2) and ((k-1)/2 + 1) which 
	  //                forces VC 0 otherwise any VC can be used
	  
	  if ( ( ( dir == 0 ) && ( cur > dest ) ) ||
	       ( ( dir == 1 ) && ( cur < dest ) ) ) {
	    *partition = 1;
	  } else if ( ( ( dir == 0 ) && ( cur <= (gK-1)/2 ) && ( dest >  (gK-1)/2 ) ) ||
		      ( ( dir == 1 ) && ( cur >  (gK-1)/2 ) && ( dest <= (gK-1)/2 ) ) ) {
	    *partition = 0;
	  } else {
	    *partition = RandomInt( 1 ); // use either VC set
	  }
	} else {
	  // Deterministic, fixed dateline between nodes k-1 and 0
	  
	  if ( ( ( dir == 0 ) && ( cur > dest ) ) ||
	       ( ( dir == 1 ) && ( dest < cur ) ) ) {
	    *partition = 1;
	  } else {
	    *partition = 0;
	  }
	}
      }
    } else {
      // Inverting the least significant bit keeps
      // the packet moving in the same direction
      *out_port = in_port ^ 0x1;
    }    

  } else {
    *out_port = 2*gN;  // Eject
  }
}

//=============================================================

void dim_order_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int out_port = inject ? -1 : dor_next_mesh( r->GetID( ), f->dest );
  
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  if ( !inject && f->watch ) {
    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
	       << "Adding VC range [" 
	       << vcBegin << "," 
	       << vcEnd << "]"
	       << " at output port " << out_port
	       << " for flit " << f->id
	       << " (input port " << in_channel
	       << ", destination " << f->dest << ")"
	       << "." << endl;
  }
  
  outputs->Clear();

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void dim_order_ni_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int out_port = inject ? -1 : dor_next_mesh( r->GetID( ), f->dest );
  
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  // at the destination router, we don't need to separate VCs by destination
  if(inject || (r->GetID() != f->dest)) {

    int const vcs_per_dest = (vcEnd - vcBegin + 1) / gNodes;
    assert(vcs_per_dest > 0);

    vcBegin += f->dest * vcs_per_dest;
    vcEnd = vcBegin + vcs_per_dest - 1;

  }
  
  if( !inject && f->watch ) {
    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
	       << "Adding VC range [" 
	       << vcBegin << "," 
	       << vcEnd << "]"
	       << " at output port " << out_port
	       << " for flit " << f->id
	       << " (input port " << in_channel
	       << ", destination " << f->dest << ")"
	       << "." << endl;
  }
  
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void dim_order_pni_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int out_port = inject ? -1 : dor_next_mesh( r->GetID(), f->dest );
  
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  if(inject || (r->GetID() != f->dest)) {
    int next_coord = f->dest;
    if(!inject) {
      int out_dim = out_port / 2;
      for(int d = 0; d < out_dim; ++d) {
	next_coord /= gK;
      }
    }
    next_coord %= gK;
    assert(next_coord >= 0 && next_coord < gK);
    int vcs_per_dest = (vcEnd - vcBegin + 1) / gK;
    assert(vcs_per_dest > 0);
    vcBegin += next_coord * vcs_per_dest;
    vcEnd = vcBegin + vcs_per_dest - 1;
  }

  if( !inject && f->watch ) {
    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
	       << "Adding VC range [" 
	       << vcBegin << "," 
	       << vcEnd << "]"
	       << " at output port " << out_port
	       << " for flit " << f->id
	       << " (input port " << in_channel
	       << ", destination " << f->dest << ")"
	       << "." << endl;
  }
  
  outputs->Clear( );
  
  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

// Random intermediate in the minimal quadrant defined
// by the source and destination
int rand_min_intr_mesh( int src, int dest )
{
  int dist;

  int intm = 0;
  int offset = 1;

  for ( int n = 0; n < gN; ++n ) {
    dist = ( dest % gK ) - ( src % gK );

    if ( dist > 0 ) {
      intm += offset * ( ( src % gK ) + RandomInt( dist ) );
    } else {
      intm += offset * ( ( dest % gK ) + RandomInt( -dist ) );
    }

    offset *= gK;
    dest /= gK; src /= gK;
  }

  return intm;
}

//=============================================================

void romm_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    if ( in_channel == 2*gN ) {
      f->ph   = 0;  // Phase 0
      f->intm = rand_min_intr_mesh( f->src, f->dest );
    } 

    if ( ( f->ph == 0 ) && ( r->GetID( ) == f->intm ) ) {
      f->ph = 1; // Go to phase 1
    }

    out_port = dor_next_mesh( r->GetID( ), (f->ph == 0) ? f->intm : f->dest );

    // at the destination router, we don't need to separate VCs by phase
    if(r->GetID() != f->dest) {

      //each class must have at least 2 vcs assigned or else valiant valiant will deadlock
      int available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      if(f->ph == 0) {
	vcEnd -= available_vcs;
      } else {
	assert(f->ph == 1);
	vcBegin += available_vcs;
      }
    }

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void romm_ni_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  // at the destination router, we don't need to separate VCs by destination
  if(inject || (r->GetID() != f->dest)) {

    int const vcs_per_dest = (vcEnd - vcBegin + 1) / gNodes;
    assert(vcs_per_dest > 0);

    vcBegin += f->dest * vcs_per_dest;
    vcEnd = vcBegin + vcs_per_dest - 1;

  }

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    if ( in_channel == 2*gN ) {
      f->ph   = 0;  // Phase 0
      f->intm = rand_min_intr_mesh( f->src, f->dest );
    } 

    if ( ( f->ph == 0 ) && ( r->GetID( ) == f->intm ) ) {
      f->ph = 1; // Go to phase 1
    }

    out_port = dor_next_mesh( r->GetID( ), (f->ph == 0) ? f->intm : f->dest );

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void min_adapt_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  outputs->Clear( );
  
  if(inject) {
    // injection can use all VCs
    outputs->AddRange(-1, vcBegin, vcEnd);
    return;
  } else if(r->GetID() == f->dest) {
    // ejection can also use all VCs
    outputs->AddRange(2*gN, vcBegin, vcEnd);
    return;
  }

  int in_vc;

  if ( in_channel == 2*gN ) {
    in_vc = vcEnd; // ignore the injection VC
  } else {
    in_vc = f->vc;
  }
  
  // DOR for the escape channel (VC 0), low priority 
  int out_port = dor_next_mesh( r->GetID( ), f->dest );    
  outputs->AddRange( out_port, 0, vcBegin, vcBegin );
  
  if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		  << "Adding VC range [" 
		  << vcBegin << "," 
		  << vcBegin << "]"
		  << " at output port " << out_port
		  << " for flit " << f->id
		  << " (input port " << in_channel
		  << ", destination " << f->dest << ")"
		  << "." << endl;
   }
  
  if ( in_vc != vcBegin ) { // If not in the escape VC
    // Minimal adaptive for all other channels
    int cur = r->GetID( );
    int dest = f->dest;
    
    for ( int n = 0; n < gN; ++n ) {
      if ( ( cur % gK ) != ( dest % gK ) ) { 
	// Add minimal direction in dimension 'n'
	if ( ( cur % gK ) < ( dest % gK ) ) { // Right
	  if ( f->watch ) {
	    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
			<< "Adding VC range [" 
		       << (vcBegin+1) << "," 
			<< vcEnd << "]"
			<< " at output port " << 2*n
			<< " with priority " << 1
			<< " for flit " << f->id
			<< " (input port " << in_channel
			<< ", destination " << f->dest << ")"
			<< "." << endl;
	  }
	  outputs->AddRange( 2*n, vcBegin+1, vcEnd, 1 ); 
	} else { // Left
	  if ( f->watch ) {
	    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
			<< "Adding VC range [" 
		       << (vcBegin+1) << "," 
			<< vcEnd << "]"
			<< " at output port " << 2*n+1
			<< " with priority " << 1
			<< " for flit " << f->id
			<< " (input port " << in_channel
			<< ", destination " << f->dest << ")"
			<< "." << endl;
	  }
	  outputs->AddRange( 2*n + 1, vcBegin+1, vcEnd, 1 ); 
	}
      }
      cur  /= gK;
      dest /= gK;
    }
  } 
}

//=============================================================

void planar_adapt_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  outputs->Clear( );
  
  if(inject) {
    // injection can use all VCs
    outputs->AddRange(-1, vcBegin, vcEnd);
    return;
  }

  int cur     = r->GetID( ); 
  int dest    = f->dest;

  if ( cur != dest ) {
   
    int in_vc   = f->vc;
    int vc_mult = (vcEnd - vcBegin + 1) / 3;

    // Find the first unmatched dimension -- except
    // for when we're in the first dimension because
    // of misrouting in the last adaptive plane.
    // In this case, go to the last dimension instead.

    int n;
    for ( n = 0; n < gN; ++n ) {
      if ( ( ( cur % gK ) != ( dest % gK ) ) &&
	   !( ( in_channel/2 == 0 ) &&
	      ( n == 0 ) &&
	      ( in_vc < vcBegin+2*vc_mult ) ) ) {
	break;
      }

      cur  /= gK;
      dest /= gK;
    }

    assert( n < gN );

    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		  << "PLANAR ADAPTIVE: flit " << f->id 
		  << " in adaptive plane " << n << "." << endl;
    }

    // We're in adaptive plane n

    // Can route productively in d_{i,2}
    bool increase;
    bool fault;
    if ( ( cur % gK ) < ( dest % gK ) ) { // Increasing
      increase = true;
      if ( !r->IsFaultyOutput( 2*n ) ) {
	outputs->AddRange( 2*n, vcBegin+2*vc_mult, vcEnd );
	fault = false;

	if ( f->watch ) {
	  *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		      << "PLANAR ADAPTIVE: increasing in dimension " << n
		      << "." << endl;
	}
      } else {
	fault = true;
      }
    } else { // Decreasing
      increase = false;
      if ( !r->IsFaultyOutput( 2*n + 1 ) ) {
	outputs->AddRange( 2*n + 1, vcBegin+2*vc_mult, vcEnd ); 
	fault = false;

	if ( f->watch ) {
	  *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		      << "PLANAR ADAPTIVE: decreasing in dimension " << n
		      << "." << endl;
	}
      } else {
	fault = true;
      }
    }
      
    n = ( n + 1 ) % gN;
    cur  /= gK;
    dest /= gK;
      
    if ( !increase ) {
      vcBegin += vc_mult;
    }
    vcEnd = vcBegin + vc_mult - 1;
      
    int d1_min_c;
    if ( ( cur % gK ) < ( dest % gK ) ) { // Increasing in d_{i+1}
      d1_min_c = 2*n;
    } else if ( ( cur % gK ) != ( dest % gK ) ) {  // Decreasing in d_{i+1}
      d1_min_c = 2*n + 1;
    } else {
      d1_min_c = -1;
    }
      
    // do we want to 180?  if so, the last
    // route was a misroute in this dimension,
    // if there is no fault in d_i, just ignore
    // this dimension, otherwise continue to misroute
    if ( d1_min_c == in_channel ) { 
      if ( fault ) {
	d1_min_c = in_channel ^ 1;
      } else {
	d1_min_c = -1;
      }

      if ( f->watch ) {
	*gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		    << "PLANAR ADAPTIVE: avoiding 180 in dimension " << n
		    << "." << endl;
      }
    }
      
    if ( d1_min_c != -1 ) {
      if ( !r->IsFaultyOutput( d1_min_c ) ) {
	outputs->AddRange( d1_min_c, vcBegin, vcEnd );
      } else if ( fault ) {
	// major problem ... fault in d_i and d_{i+1}
	r->Error( "There seem to be faults in d_i and d_{i+1}" );
      }
    } else if ( fault ) { // need to misroute!
      bool atedge;
      if ( cur % gK == 0 ) {
	d1_min_c = 2*n;
	atedge = true;
      } else if ( cur % gK == gK - 1 ) {
	d1_min_c = 2*n + 1;
	atedge = true;
      } else {
	d1_min_c = 2*n + RandomInt( 1 ); // random misroute

	if ( d1_min_c  == in_channel ) { // don't 180
	  d1_min_c = in_channel ^ 1;
	}
	atedge = false;
      }
      
      if ( !r->IsFaultyOutput( d1_min_c ) ) {
	outputs->AddRange( d1_min_c, vcBegin, vcEnd );
      } else if ( !atedge && !r->IsFaultyOutput( d1_min_c ^ 1 ) ) {
	outputs->AddRange( d1_min_c ^ 1, vcBegin, vcEnd );
      } else {
	// major problem ... fault in d_i and d_{i+1}
	r->Error( "There seem to be faults in d_i and d_{i+1}" );
      }
    }
  } else {
    outputs->AddRange( 2*gN, vcBegin, vcEnd ); 
  }
}

//=============================================================
/*
  FIXME: This is broken (note that f->dr is never actually modified).
  Even if it were, this should really use f->ph instead of introducing a single-
  use field.

void limited_adapt_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  outputs->Clear( );

  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  if ( inject ) {
    outputs->AddRange( -1, vcBegin, vcEnd - 1 );
    f->dr = 0; // zero dimension reversals
    return;
  }

  int cur = r->GetID( );
  int dest = f->dest;
  
  if ( cur != dest ) {
    if ( ( f->vc != vcEnd ) && 
	 ( f->dr != vcEnd - 1 ) ) {
      
      for ( int n = 0; n < gN; ++n ) {
	if ( ( cur % gK ) != ( dest % gK ) ) { 
	  int min_port;
	  if ( ( cur % gK ) < ( dest % gK ) ) { 
	    min_port = 2*n; // Right
	  } else {
	    min_port = 2*n + 1; // Left
	  }
	  
	  // Go in a productive direction with high priority
	  outputs->AddRange( min_port, vcBegin, vcEnd - 1, 2 );
	  
	  // Go in the non-productive direction with low priority
	  outputs->AddRange( min_port ^ 0x1, vcBegin, vcEnd - 1, 1 );
	} else {
	  // Both directions are non-productive
	  outputs->AddRange( 2*n, vcBegin, vcEnd - 1, 1 );
	  outputs->AddRange( 2*n+1, vcBegin, vcEnd - 1, 1 );
	}
	
	cur  /= gK;
	dest /= gK;
      }
      
    } else {
      outputs->AddRange( dor_next_mesh( cur, dest ),
			 vcEnd, vcEnd, 0 );
    }
    
  } else { // at destination
    outputs->AddRange( 2*gN, vcBegin, vcEnd ); 
  }
}
*/
//=============================================================

void valiant_mesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    if ( in_channel == 2*gN ) {
      f->ph   = 0;  // Phase 0
      f->intm = RandomInt( gNodes - 1 );
    }

    if ( ( f->ph == 0 ) && ( r->GetID( ) == f->intm ) ) {
      f->ph = 1; // Go to phase 1
    }

    out_port = dor_next_mesh( r->GetID( ), (f->ph == 0) ? f->intm : f->dest );

    // at the destination router, we don't need to separate VCs by phase
    if(r->GetID() != f->dest) {

      //each class must have at least 2 vcs assigned or else valiant valiant will deadlock
      int const available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      if(f->ph == 0) {
	vcEnd -= available_vcs;
      } else {
	assert(f->ph == 1);
	vcBegin += available_vcs;
      }
    }

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void valiant_torus( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int phase;
    if ( in_channel == 2*gN ) {
      phase   = 0;  // Phase 0
      f->intm = RandomInt( gNodes - 1 );
    } else {
      phase = f->ph / 2;
    }

    if ( ( phase == 0 ) && ( r->GetID( ) == f->intm ) ) {
      phase = 1; // Go to phase 1
      in_channel = 2*gN; // ensures correct vc selection at the beginning of phase 2
    }
  
    int ring_part;
    dor_next_torus( r->GetID( ), (phase == 0) ? f->intm : f->dest, in_channel,
		    &out_port, &ring_part, false );

    f->ph = 2 * phase + ring_part;

    // at the destination router, we don't need to separate VCs by phase, etc.
    if(r->GetID() != f->dest) {

      int const ring_available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(ring_available_vcs > 0);

      if(ring_part == 0) {
	vcEnd -= ring_available_vcs;
      } else {
	assert(ring_part == 1);
	vcBegin += ring_available_vcs;
      }

      int const ph_available_vcs = ring_available_vcs / 2;
      assert(ph_available_vcs > 0);

      if(phase == 0) {
	vcEnd -= ph_available_vcs;
      } else {
	assert(phase == 1);
	vcBegin += ph_available_vcs;
      }
    }

  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void valiant_ni_torus( const Router *r, const Flit *f, int in_channel, 
		       OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  // at the destination router, we don't need to separate VCs by destination
  if(inject || (r->GetID() != f->dest)) {

    int const vcs_per_dest = (vcEnd - vcBegin + 1) / gNodes;
    assert(vcs_per_dest > 0);

    vcBegin += f->dest * vcs_per_dest;
    vcEnd = vcBegin + vcs_per_dest - 1;

  }

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int phase;
    if ( in_channel == 2*gN ) {
      phase   = 0;  // Phase 0
      f->intm = RandomInt( gNodes - 1 );
    } else {
      phase = f->ph / 2;
    }

    if ( ( f->ph == 0 ) && ( r->GetID( ) == f->intm ) ) {
      f->ph = 1; // Go to phase 1
      in_channel = 2*gN; // ensures correct vc selection at the beginning of phase 2
    }
  
    int ring_part;
    dor_next_torus( r->GetID( ), (f->ph == 0) ? f->intm : f->dest, in_channel,
		    &out_port, &ring_part, false );

    f->ph = 2 * phase + ring_part;

    // at the destination router, we don't need to separate VCs by phase, etc.
    if(r->GetID() != f->dest) {

      int const ring_available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(ring_available_vcs > 0);

      if(ring_part == 0) {
	vcEnd -= ring_available_vcs;
      } else {
	assert(ring_part == 1);
	vcBegin += ring_available_vcs;
      }

      int const ph_available_vcs = ring_available_vcs / 2;
      assert(ph_available_vcs > 0);

      if(phase == 0) {
	vcEnd -= ph_available_vcs;
      } else {
	assert(phase == 1);
	vcBegin += ph_available_vcs;
      }
    }

    if (f->watch) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		 << "Adding VC range [" 
		 << vcBegin << "," 
		 << vcEnd << "]"
		 << " at output port " << out_port
		 << " for flit " << f->id
		 << " (input port " << in_channel
		 << ", destination " << f->dest << ")"
		 << "." << endl;
    }

  }
  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void dim_order_torus( const Router *r, const Flit *f, int in_channel, 
		      OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int cur  = r->GetID( );
    int dest = f->dest;

    dor_next_torus( cur, dest, in_channel,
		    &out_port, &f->ph, false );


    // at the destination router, we don't need to separate VCs by ring partition
    if(cur != dest) {

      int const available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      if ( f->ph == 0 ) {
	vcEnd -= available_vcs;
      } else {
	vcBegin += available_vcs;
      } 
    }

    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		 << "Adding VC range [" 
		 << vcBegin << "," 
		 << vcEnd << "]"
		 << " at output port " << out_port
		 << " for flit " << f->id
		 << " (input port " << in_channel
		 << ", destination " << f->dest << ")"
		 << "." << endl;
    }

  }
 
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void dim_order_ni_torus( const Router *r, const Flit *f, int in_channel, 
			 OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {
    
    int cur  = r->GetID( );
    int dest = f->dest;

    dor_next_torus( cur, dest, in_channel,
		    &out_port, NULL, false );

    // at the destination router, we don't need to separate VCs by destination
    if(cur != dest) {

      int const vcs_per_dest = (vcEnd - vcBegin + 1) / gNodes;
      assert(vcs_per_dest);

      vcBegin += f->dest * vcs_per_dest;
      vcEnd = vcBegin + vcs_per_dest - 1;

    }

    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		 << "Adding VC range [" 
		 << vcBegin << "," 
		 << vcEnd << "]"
		 << " at output port " << out_port
		 << " for flit " << f->id
		 << " (input port " << in_channel
		 << ", destination " << f->dest << ")"
		 << "." << endl;
    }

  }
  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void dim_order_bal_torus( const Router *r, const Flit *f, int in_channel, 
			  OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int cur  = r->GetID( );
    int dest = f->dest;

    dor_next_torus( cur, dest, in_channel,
		    &out_port, &f->ph, true );

    // at the destination router, we don't need to separate VCs by ring partition
    if(cur != dest) {

      int const available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      if ( f->ph == 0 ) {
	vcEnd -= available_vcs;
      } else {
	assert(f->ph == 1);
	vcBegin += available_vcs;
      } 
    }

    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
		 << "Adding VC range [" 
		 << vcBegin << "," 
		 << vcEnd << "]"
		 << " at output port " << out_port
		 << " for flit " << f->id
		 << " (input port " << in_channel
		 << ", destination " << f->dest << ")"
		 << "." << endl;
    }

  }
  
  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}

//=============================================================

void min_adapt_torus( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  outputs->Clear( );

  if(inject) {
    // injection can use all VCs
    outputs->AddRange(-1, vcBegin, vcEnd);
    return;
  } else if(r->GetID() == f->dest) {
    // ejection can also use all VCs
    outputs->AddRange(2*gN, vcBegin, vcEnd);
  }

  int in_vc;
  if ( in_channel == 2*gN ) {
    in_vc = vcEnd; // ignore the injection VC
  } else {
    in_vc = f->vc;
  }
  
  int cur = r->GetID( );
  int dest = f->dest;

  int out_port;

  if ( in_vc > ( vcBegin + 1 ) ) { // If not in the escape VCs
    // Minimal adaptive for all other channels
    
    for ( int n = 0; n < gN; ++n ) {
      if ( ( cur % gK ) != ( dest % gK ) ) {
	int dist2 = gK - 2 * ( ( ( dest % gK ) - ( cur % gK ) + gK ) % gK );
	
	if ( dist2 > 0 ) { /*) || 
			     ( ( dist2 == 0 ) && ( RandomInt( 1 ) ) ) ) {*/
	  outputs->AddRange( 2*n, vcBegin+3, vcBegin+3, 1 ); // Right
	} else {
	  outputs->AddRange( 2*n + 1, vcBegin+3, vcBegin+3, 1 ); // Left
	}
      }

      cur  /= gK;
      dest /= gK;
    }
    
    // DOR for the escape channel (VCs 0-1), low priority --- 
    // trick the algorithm with the in channel.  want VC assignment
    // as if we had injected at this node
    dor_next_torus( r->GetID( ), f->dest, 2*gN,
		    &out_port, &f->ph, false );
  } else {
    // DOR for the escape channel (VCs 0-1), low priority 
    dor_next_torus( cur, dest, in_channel,
		    &out_port, &f->ph, false );
  }

  if ( f->ph == 0 ) {
    outputs->AddRange( out_port, vcBegin, vcBegin, 0 );
  } else  {
    outputs->AddRange( out_port, vcBegin+1, vcBegin+1, 0 );
  } 
}

//=============================================================

void dest_tag_fly( const Router *r, const Flit *f, int in_channel, 
		   OutputSet *outputs, bool inject )
{
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    int stage = ( r->GetID( ) * gK ) / gNodes;
    int dest  = f->dest;

    while( stage < ( gN - 1 ) ) {
      dest /= gK;
      ++stage;
    }

    out_port = dest % gK;
  }

  outputs->Clear( );

  outputs->AddRange( out_port, vcBegin, vcEnd );
}



//=============================================================

void chaos_torus( const Router *r, const Flit *f, 
		  int in_channel, OutputSet *outputs, bool inject )
{
  outputs->Clear( );

  if(inject) {
    outputs->AddRange(-1, 0, 0);
    return;
  }

  int cur = r->GetID( );
  int dest = f->dest;
  
  if ( cur != dest ) {
    for ( int n = 0; n < gN; ++n ) {

      if ( ( cur % gK ) != ( dest % gK ) ) { 
	int dist2 = gK - 2 * ( ( ( dest % gK ) - ( cur % gK ) + gK ) % gK );
      
	if ( dist2 >= 0 ) {
	  outputs->AddRange( 2*n, 0, 0 ); // Right
	} 
	
	if ( dist2 <= 0 ) {
	  outputs->AddRange( 2*n + 1, 0, 0 ); // Left
	}
      }

      cur  /= gK;
      dest /= gK;
    }
  } else {
    outputs->AddRange( 2*gN, 0, 0 ); 
  }
}


//=============================================================

void chaos_mesh( const Router *r, const Flit *f, 
		  int in_channel, OutputSet *outputs, bool inject )
{
  outputs->Clear( );

  if(inject) {
    outputs->AddRange(-1, 0, 0);
    return;
  }

  int cur = r->GetID( );
  int dest = f->dest;
  
  if ( cur != dest ) {
    for ( int n = 0; n < gN; ++n ) {
      if ( ( cur % gK ) != ( dest % gK ) ) { 
	// Add minimal direction in dimension 'n'
	if ( ( cur % gK ) < ( dest % gK ) ) { // Right
	  outputs->AddRange( 2*n, 0, 0 ); 
	} else { // Left
	  outputs->AddRange( 2*n + 1, 0, 0 ); 
	}
      }
      cur  /= gK;
      dest /= gK;
    }
  } else {
    outputs->AddRange( 2*gN, 0, 0 ); 
  }
}

//=============================================================

void InitializeRoutingMap( const Configuration & config )
{

  gNumVCs = config.GetInt( "num_vcs" );

  //
  // traffic class partitions
  //
  gReadReqBeginVC    = config.GetInt("read_request_begin_vc");
  if(gReadReqBeginVC < 0) {
    gReadReqBeginVC = 0;
  }
  gReadReqEndVC      = config.GetInt("read_request_end_vc");
  if(gReadReqEndVC < 0) {
    gReadReqEndVC = gNumVCs / 2 - 1;
  }
  gWriteReqBeginVC   = config.GetInt("write_request_begin_vc");
  if(gWriteReqBeginVC < 0) {
    gWriteReqBeginVC = 0;
  }
  gWriteReqEndVC     = config.GetInt("write_request_end_vc");
  if(gWriteReqEndVC < 0) {
    gWriteReqEndVC = gNumVCs / 2 - 1;
  }
  gReadReplyBeginVC  = config.GetInt("read_reply_begin_vc");
  if(gReadReplyBeginVC < 0) {
    gReadReplyBeginVC = gNumVCs / 2;
  }
  gReadReplyEndVC    = config.GetInt("read_reply_end_vc");
  if(gReadReplyEndVC < 0) {
    gReadReplyEndVC = gNumVCs - 1;
  }
  gWriteReplyBeginVC = config.GetInt("write_reply_begin_vc");
  if(gWriteReplyBeginVC < 0) {
    gWriteReplyBeginVC = gNumVCs / 2;
  }
  gWriteReplyEndVC   = config.GetInt("write_reply_end_vc");
  if(gWriteReplyEndVC < 0) {
    gWriteReplyEndVC = gNumVCs - 1;
  }

  /* Register routing functions here */

  // ===================================================
  // Balfour-Schultz
  gRoutingFunctionMap["nca_fattree"]         = &fattree_nca;
  gRoutingFunctionMap["anca_fattree"]        = &fattree_anca;
  gRoutingFunctionMap["nca_qtree"]           = &qtree_nca;
  gRoutingFunctionMap["nca_tree4"]           = &tree4_nca;
  gRoutingFunctionMap["anca_tree4"]          = &tree4_anca;
  gRoutingFunctionMap["dor_mesh"]            = &dim_order_mesh;
  gRoutingFunctionMap["xy_yx_mesh"]          = &xy_yx_mesh;
  gRoutingFunctionMap["adaptive_xy_yx_mesh"] = &adaptive_xy_yx_mesh;
  // End Balfour-Schultz

  // Kasan
  gRoutingFunctionMap["anca_greedy_fattree"]              = &fattree_anca_greedy;
  gRoutingFunctionMap["nca_sourcehash_fattree"]           = &fattree_nca_sourcehash;
  gRoutingFunctionMap["nca_desthash_fattree"]             = &fattree_nca_desthash;
  gRoutingFunctionMap["nca_roundrobin_fattree"]           = &fattree_nca_roundrobin;
  gRoutingFunctionMap["nca_fibonaccihash_fattree"]        = &fattree_nca_fibonaccihash;
  gRoutingFunctionMap["nca_fibonacci_pid_hash_fattree"]   = &fattree_nca_fibonacci_pid_hash;
  gRoutingFunctionMap["nca_fibonacci_inj_hash_fattree"]   = &fattree_nca_fibonacci_inj_hash;
  // End Kasan
  // ===================================================

  // Tho
  gRoutingFunctionMap["nca_fattree2"]                     = &fattree2_nca;
  gRoutingFunctionMap["nca_hybrid_fattree2"]              = &fattree2_nca_hybrid;

  gRoutingFunctionMap["nca_xorhash_fattree"]              = &fattree_nca_xorhash;

  gRoutingFunctionMap["nca_hybrid_fattree"]               = &fattree_nca_hybrid;
  gRoutingFunctionMap["nca_hybrid_mem_fattree"]           = &fattree_nca_hybrid_mem;
  gRoutingFunctionMap["nca_portlimit_fattree"]            = &fattree_nca_portlimit;
  gRoutingFunctionMap["nca_fibonacci_fattree"]            = &fattree_nca_fibonacci;
  gRoutingFunctionMap["nca_murmurhash3_fattree"]          = &fattree_nca_murmurhash3;
  gRoutingFunctionMap["nca_crc_fattree"]                  = &fattree_nca_crc;
  // End Tho
  // ===================================================


  gRoutingFunctionMap["dim_order_mesh"]      = &dim_order_mesh;
  gRoutingFunctionMap["dim_order_ni_mesh"]   = &dim_order_ni_mesh;
  gRoutingFunctionMap["dim_order_pni_mesh"]  = &dim_order_pni_mesh;
  gRoutingFunctionMap["dim_order_torus"]     = &dim_order_torus;
  gRoutingFunctionMap["dim_order_ni_torus"]  = &dim_order_ni_torus;
  gRoutingFunctionMap["dim_order_bal_torus"] = &dim_order_bal_torus;

  gRoutingFunctionMap["romm_mesh"]           = &romm_mesh; 
  gRoutingFunctionMap["romm_ni_mesh"]        = &romm_ni_mesh;

  gRoutingFunctionMap["min_adapt_mesh"]      = &min_adapt_mesh;
  gRoutingFunctionMap["min_adapt_torus"]     = &min_adapt_torus;

  gRoutingFunctionMap["planar_adapt_mesh"]   = &planar_adapt_mesh;

  // FIXME: This is broken.
  //  gRoutingFunctionMap["limited_adapt_mesh"] = &limited_adapt_mesh;

  gRoutingFunctionMap["valiant_mesh"]        = &valiant_mesh;
  gRoutingFunctionMap["valiant_torus"]       = &valiant_torus;
  gRoutingFunctionMap["valiant_ni_torus"]    = &valiant_ni_torus;

  gRoutingFunctionMap["dest_tag_fly"]        = &dest_tag_fly;

  gRoutingFunctionMap["chaos_mesh"]          = &chaos_mesh;
  gRoutingFunctionMap["chaos_torus"]         = &chaos_torus;
}
