
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Derived from code by Intan Technologies, LLC
// 
// Module Name:    CIPO_phase_selector 
// Description:    Downsamples CIPO by factor of 4, with selectable phase lag to
//                 compensate for headstage cable delay.
//                 Delivers a double-data-rate data word as high 16 bits of
//                 results register
//
//
//////////////////////////////////////////////////////////////////////////////////
module CIPO_combined_phase_selector(
	input wire [3:0] 	phase_select,	// CIPO sampling phase lag to compensate for headstage cable delay
	input wire [73:0] 	CIPO4x,			// 4x oversampled CIPO input
	output reg [31:0] 	CIPO			// 32-bit CIPO output [31:16] = DDR, [15:0] = regular
	);

	// DDR phase data (upper 16 bits) - offset by 2 phases
	always @(*) begin
		case (phase_select)
			0:       CIPO[31:16] <= {CIPO4x[2],  CIPO4x[6],  CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62]};
			1:       CIPO[31:16] <= {CIPO4x[3],  CIPO4x[7],  CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63]};
			2:       CIPO[31:16] <= {CIPO4x[4],  CIPO4x[8],  CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60], CIPO4x[64]};
			3:       CIPO[31:16] <= {CIPO4x[5],  CIPO4x[9],  CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65]};
			4:       CIPO[31:16] <= {CIPO4x[6],  CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62], CIPO4x[66]};
			5:       CIPO[31:16] <= {CIPO4x[7],  CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63], CIPO4x[67]};
			6:       CIPO[31:16] <= {CIPO4x[8],  CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60], CIPO4x[64], CIPO4x[68]};
			7:       CIPO[31:16] <= {CIPO4x[9],  CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65], CIPO4x[69]};
			8:       CIPO[31:16] <= {CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62], CIPO4x[66], CIPO4x[70]};
			9:       CIPO[31:16] <= {CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63], CIPO4x[67], CIPO4x[71]};
			10:      CIPO[31:16] <= {CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60], CIPO4x[64], CIPO4x[68], CIPO4x[72]};
			11:      CIPO[31:16] <= {CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65], CIPO4x[69], CIPO4x[73]};
			default: CIPO[31:16] <= {CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65], CIPO4x[69], CIPO4x[73]};
		endcase
	end

	// Regular phase data (lower 16 bits)
	always @(*) begin
		case (phase_select)
			0:       CIPO[15:0] <= {CIPO4x[0],  CIPO4x[4],  CIPO4x[8],  CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60]};
			1:       CIPO[15:0] <= {CIPO4x[1],  CIPO4x[5],  CIPO4x[9],  CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61]};
			2:       CIPO[15:0] <= {CIPO4x[2],  CIPO4x[6],  CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62]};
			3:       CIPO[15:0] <= {CIPO4x[3],  CIPO4x[7],  CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63]};
			4:       CIPO[15:0] <= {CIPO4x[4],  CIPO4x[8],  CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60], CIPO4x[64]};
			5:       CIPO[15:0] <= {CIPO4x[5],  CIPO4x[9],  CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65]};
			6:       CIPO[15:0] <= {CIPO4x[6],  CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62], CIPO4x[66]};
			7:       CIPO[15:0] <= {CIPO4x[7],  CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63], CIPO4x[67]};
			8:       CIPO[15:0] <= {CIPO4x[8],  CIPO4x[12], CIPO4x[16], CIPO4x[20], CIPO4x[24], CIPO4x[28], CIPO4x[32], CIPO4x[36], CIPO4x[40], CIPO4x[44], CIPO4x[48], CIPO4x[52], CIPO4x[56], CIPO4x[60], CIPO4x[64], CIPO4x[68]};
			9:       CIPO[15:0] <= {CIPO4x[9],  CIPO4x[13], CIPO4x[17], CIPO4x[21], CIPO4x[25], CIPO4x[29], CIPO4x[33], CIPO4x[37], CIPO4x[41], CIPO4x[45], CIPO4x[49], CIPO4x[53], CIPO4x[57], CIPO4x[61], CIPO4x[65], CIPO4x[69]};
			10:      CIPO[15:0] <= {CIPO4x[10], CIPO4x[14], CIPO4x[18], CIPO4x[22], CIPO4x[26], CIPO4x[30], CIPO4x[34], CIPO4x[38], CIPO4x[42], CIPO4x[46], CIPO4x[50], CIPO4x[54], CIPO4x[58], CIPO4x[62], CIPO4x[66], CIPO4x[70]};
			11:      CIPO[15:0] <= {CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63], CIPO4x[67], CIPO4x[71]};
			default: CIPO[15:0] <= {CIPO4x[11], CIPO4x[15], CIPO4x[19], CIPO4x[23], CIPO4x[27], CIPO4x[31], CIPO4x[35], CIPO4x[39], CIPO4x[43], CIPO4x[47], CIPO4x[51], CIPO4x[55], CIPO4x[59], CIPO4x[63], CIPO4x[67], CIPO4x[71]};
		endcase
	end
	
endmodule
