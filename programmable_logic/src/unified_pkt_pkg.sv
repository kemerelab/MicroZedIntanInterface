// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// unified_pkt_pkg.sv
//
// The wire-format contract for every PL->host stream, in one place.
//
// All streams (broadband, LFP, ...) leave the board on ONE UDP port and are told
// apart by `stream_type` in header word 1, so every stream MUST emit the same
// 8-word common header ahead of its payload. That header is defined here and
// nowhere else: it is read by the firmware, `remote/net.py`, and the Open Ephys
// plugin, so a silent divergence between two producers is a wire-format bug that
// no single module can catch. Anything that builds a packet imports this package
// rather than restating the magic number or the word order.
//
// Header layout (32-bit little-endian words):
//
//   w0  MAGIC     0xCAFEBABE -- how the host finds a frame start
//   w1  TYPE_VER  stream_type[7:0] | version[15:8] | flags[31:16]
//   w2  TS_LO     64-bit master sample timestamp, low word
//   w3  TS_HI     ... high word
//   w4  SEQ       per-stream sequence number, +1 per emitted frame. THE loss
//                 check: the host flags any gap, so loss is proven, not assumed.
//   w5  AUX0      stream-specific
//   w6  AUX1      stream-specific
//   w7  RSVD      0
//
// Words 0-4 and 7 are identical for every stream; only AUX0/AUX1 belong to the
// individual stream. Producers differ in how WIDE they write (broadband packs
// 64-bit pairs into its FIFO, the LFP engine writes 32-bit words straight to
// BRAM), so both a word-at-a-time accessor and a whole-header vector are
// provided -- the transport differs, the contract does not.
//
// See docs/unified-packet-format.md for the prose version and the payload
// layouts, and docs/register-map.md for the control/status registers.

package unified_pkt_pkg;

    // Header word 0, every stream.
    localparam logic [31:0] UNIFIED_MAGIC = 32'hCAFEBABE;

    // Bumped when the header LAYOUT changes -- not when a stream type is added.
    localparam logic [7:0] UNIFIED_VERSION = 8'd1;

    // Common header length in 32-bit words, emitted before every payload.
    localparam int UNIFIED_HDR_WORDS = 8;

    // stream_type (header word 1, [7:0]) -- what the host demuxes the port on.
    localparam logic [7:0] STREAM_TYPE_BROADBAND = 8'd1;   // 30 kHz amplifier stream
    localparam logic [7:0] STREAM_TYPE_LFP       = 8'd2;   // decimated LFP band
    localparam logic [7:0] STREAM_TYPE_WAVELET   = 8'd3;   // reserved: on-PL scalogram

    // Header word 1. `flags` is reserved: no stream sets it today, and the host
    // ignores it, so it is free for a future per-stream marker.
    function automatic logic [31:0] unified_type_ver(input logic [7:0]  stream_type,
                                                     input logic [15:0] flags);
        return {flags, UNIFIED_VERSION, stream_type};
    endfunction

    // One header word by index, for producers that write 32 bits at a time.
    function automatic logic [31:0] unified_hdr_word(
            input logic [2:0]  idx,
            input logic [7:0]  stream_type,
            input logic [63:0] timestamp,
            input logic [31:0] seq,
            input logic [31:0] aux0,
            input logic [31:0] aux1);
        case (idx)
            3'd0:    unified_hdr_word = UNIFIED_MAGIC;
            3'd1:    unified_hdr_word = unified_type_ver(stream_type, 16'd0);
            3'd2:    unified_hdr_word = timestamp[31:0];
            3'd3:    unified_hdr_word = timestamp[63:32];
            3'd4:    unified_hdr_word = seq;
            3'd5:    unified_hdr_word = aux0;
            3'd6:    unified_hdr_word = aux1;
            default: unified_hdr_word = 32'd0;             // w7 RSVD
        endcase
    endfunction

    // The whole header as one packed vector, word 0 in the LOW bits, for
    // producers that write wider than 32 bits: a 64-bit writer takes
    // hdr[63:0] = {w1,w0}, hdr[127:64] = {w3,w2}, and so on.
    function automatic logic [UNIFIED_HDR_WORDS*32-1:0] unified_hdr(
            input logic [7:0]  stream_type,
            input logic [63:0] timestamp,
            input logic [31:0] seq,
            input logic [31:0] aux0,
            input logic [31:0] aux1);
        for (int i = 0; i < UNIFIED_HDR_WORDS; i++)
            unified_hdr[i*32 +: 32] =
                unified_hdr_word(i[2:0], stream_type, timestamp, seq, aux0, aux1);
    endfunction

endpackage
