// Copyright 2024 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// SPDX-License-Identifier: SHL-0.51

// Author: Maurus Item <itemm@student.ethz.ch>
//
// Description Aux chain for FPNew, handles transmitting of shared handshake and aux data
// And enables the correct lanes so they always stay in sync.
// This version can be used for lanes that have some form of FSM in them and only eventually are ready

`include "common_cells/registers.svh"

module fpnew_aux_fsm #(
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::BEFORE,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic
) (
  input logic                                  clk_i,
  input logic                                  rst_ni,
  // Input signals
  input TagType                                tag_i,
  input AuxType                                aux_i,
  // Input Handshake
  input  logic                                 in_valid_i,
  output logic                                 in_ready_o,
  input  logic                                 flush_i,
  // Output signals
  output TagType                               tag_o,
  output AuxType                               aux_o,
  // Output handshake
  output logic                                 out_valid_o,
  input  logic                                 out_ready_i,
  // Register Enable for Lanes
  output logic [NumPipeRegs-1:0]               reg_enable_o,
  // Signals for the Lane FSMs
  output logic                                 fsm_start_o,
  input logic                                  fsm_ready_i,
  // Indication of valid data in flight
  output logic                                 busy_o
);

  // ----------
  // Pipeline Distribution
  // ----------
  // This must match between this module and modules that use this module as reg enable input!
  localparam NUM_INP_REGS = (PipeConfig == fpnew_pkg::BEFORE)
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 2) // Last to get distributed regs
                               : 0); // Always have one reg to use for FSM Input
  localparam NUM_OUT_REGS = (PipeConfig == fpnew_pkg::AFTER || PipeConfig == fpnew_pkg::INSIDE)
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 2) // First to get distributed regs
                               : 0); // no regs here otherwise

  // ---------------
  // Input pipeline
  // ---------------
  // Input pipeline signals, index i holds signal after i register stages
  TagType [0:NUM_INP_REGS]               in_tag;
  AuxType [0:NUM_INP_REGS]               in_aux;
  logic   [0:NUM_INP_REGS]               in_valid;

  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] in_ready;

  // First element of pipeline is taken from inputs
  assign in_tag        [0] = tag_i;
  assign in_aux        [0] = aux_i;
  assign in_valid      [0] = in_valid_i;

  // Propagate pipeline ready signal to upstream circuitry
  assign in_ready_o = in_ready[0];

  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign in_ready[i] = in_ready[i+1] | ~in_valid[i+1];

    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(in_valid[i+1], in_valid[i], in_ready[i], flush_i, 1'b0, clk_i, rst_ni)

    // Enable register if pipleine ready and a valid data item is present
    assign reg_enable_o[i] = in_ready[i] & in_valid[i];

    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(        in_tag[i+1],         in_tag[i], reg_enable_o[i], TagType'('0))
    `FFL(        in_aux[i+1],         in_aux[i], reg_enable_o[i], AuxType'('0))
  end

  // ----------
  // Global FSM
  // ----------

  // Input & Output Handshake
  logic fsm_in_valid, fsm_in_ready;
  logic fsm_out_valid, fsm_out_ready;
  logic in_handshake, out_handshake;

  // State for FSM
  logic fsm_busy_d, fsm_busy_q;

  // Upstream Handshake Connection
  assign fsm_in_valid = in_valid[NUM_INP_REGS];
  assign in_ready[NUM_INP_REGS] = fsm_in_ready;

  // Assign Helper signals
  assign in_handshake = fsm_in_valid & fsm_in_ready;
  assign out_handshake = fsm_out_valid & fsm_out_ready;

  // Same as normal pipeline stage, but additionally fsm_ready_i needs to be 1.
  assign fsm_in_ready = fsm_ready_i & (fsm_out_ready | ~fsm_busy_q);

  // Start FSM on handshake
  assign fsm_start_o = in_handshake;

  // When we hold data and lanes are done/ready output it
  assign fsm_out_valid = fsm_busy_q & fsm_ready_i;

  // Change state if handshake are not the same
  // In 1 Out 0 -> We have data now
  // In 0 Out 1 -> Data is now gone
  assign fsm_busy_d = (in_handshake == out_handshake) ? fsm_busy_q : in_handshake;

  `FF(fsm_busy_q, fsm_busy_d, 1'b0);

  // ----------------
  // Data Holding FFs
  // ----------------

  // Data holding signals
  TagType held_tag, fsm_out_tag;
  AuxType held_aux, fsm_out_aux;

  `FFL(        held_tag,         in_tag[NUM_INP_REGS], fsm_start_o, TagType'('0));
  `FFL(        held_aux,         in_aux[NUM_INP_REGS], fsm_start_o, AuxType'('0));

  // Bypass in case result gets used immediately
  assign fsm_out_tag = fsm_start_o ?  in_tag[NUM_INP_REGS] : held_tag;
  assign fsm_out_aux = fsm_start_o ?  in_aux[NUM_INP_REGS] : held_aux;

  // ---------------
  // Output pipeline
  // ---------------

  // Output pipeline signals, index i holds signal after i register stages
  TagType [0:NUM_OUT_REGS]               out_tag;
  AuxType [0:NUM_OUT_REGS]               out_aux;
  logic   [0:NUM_OUT_REGS]               out_valid;

  // Ready signal is combinatorial for all stages
  logic   [0:NUM_OUT_REGS]               out_ready;

  // Connect to upstream Handshake
  assign out_valid[0] = fsm_out_valid;
  assign fsm_out_ready = out_ready[0];

  // Connect to Hold Register
  assign out_tag        [0] = fsm_out_tag;
  assign out_aux        [0] = fsm_out_aux;

  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline

    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_ready[i] = out_ready[i+1] | ~out_valid[i+1];

    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_valid[i+1], out_valid[i], out_ready[i], flush_i, 1'b0, clk_i, rst_ni)

    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_ready[i] & out_valid[i];

    // Drive external registers with reg enable
    assign reg_enable_o[NUM_INP_REGS + i] = reg_ena;

    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(        out_tag[i+1],         out_tag[i], reg_ena, TagType'('0))
    `FFL(        out_aux[i+1],         out_aux[i], reg_ena, AuxType'('0))
  end

  // Ready travels backwards from output side, driven by downstream circuitry
  assign out_ready[NUM_OUT_REGS] = out_ready_i;

  // Assign module outputs
  assign tag_o           = out_tag        [NUM_OUT_REGS];
  assign aux_o           = out_aux        [NUM_OUT_REGS];
  assign out_valid_o     = out_valid      [NUM_OUT_REGS];

  // Assign output Flags: Busy if any element inside the pipe is valid
  assign busy_o          = |in_valid | |out_valid | fsm_busy_q;

endmodule
