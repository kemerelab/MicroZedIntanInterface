#!/usr/bin/env python3
# Bit-exact reference for the full CIC^4(/5) -> comp-FIR halfband(/2) = /10 chain,
# per channel. Models exactly what cic_decimator + lfp_halfband compute, so the
# chained TB (cic_chain_tb.sv) can verify the wired datapath end-to-end.
#
# Stage 1 (CIC): identical to gen_cic_vectors.py (modular ACC_W integrators/combs,
#   >> GAIN_SHIFT, saturate to int16).
# Stage 2 (halfband FIR /2): identical to gen_halfband_vectors.py (integer MAC,
#   round-to-nearest, >> COEF_FRAC, saturate int16) on the CIC outputs.
import math, os
OUT = os.path.dirname(os.path.abspath(__file__))

N_LANES, N_SLOTS = 8, 32
DATA_W = 16
# CIC
R_CIC, N_ORDER, ACC_W, GAIN_SHIFT = 5, 4, 32, 10
# halfband comp-FIR
COEF_W, COEF_FRAC, OUT_W = 18, 17, 16
HB_TAPS = 43
HB_RING = 64
# stimulus
K_PACKETS = 200          # 30 kHz input ticks; /5 -> 40 CIC outs; /2 -> 20 chain outs
LANE_MASK = 0b1010_0101

ACC_MASK = (1 << ACC_W) - 1
def sx(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if (v >> (w-1)) & 1 else v
def to_hex(v, bits): return format(v & ((1 << bits)-1), 'x').zfill((bits+3)//4)

# ---- comp-FIR designer (droop-comp, frequency sampling) -- must match net.py ----
FS_IN = 30000.0; FS1 = FS_IN/R_CIC
def cic_mag(f):
    w = math.pi*f/FS_IN
    if abs(w) < 1e-12: return 1.0
    d = R_CIC*math.sin(w)
    return (math.sin(R_CIC*w)/d)**N_ORDER if abs(d) > 1e-15 else 1.0
CIC_DC = (R_CIC**N_ORDER)/(1 << GAIN_SHIFT)
def i0(x):
    s,t,k=1.0,1.0,0
    while True:
        k+=1;t*=(x*x)/(4*k*k);s+=t
        if t<1e-12*s:return s
def kaiser(N,b):
    a=(N-1)/2.0;return [i0(b*math.sqrt(1-((n-a)/a)**2))/i0(b) for n in range(N)]
def design_comp_fir(N=HB_TAPS, fc=1300.0, beta=6.0):
    M=N-1; win=kaiser(N,beta); a=M/2.0
    def desired(f):
        if f>fc: return 0.0
        dr=cic_mag(f); return (1.0/dr) if dr>1e-6 else 1.0
    L=2048; fs=[FS1/2*k/L for k in range(L+1)]; Hd=[desired(f) for f in fs]
    df=fs[1]-fs[0]; h=[0.0]*N
    for n in range(N):
        acc=0.0
        for k in range(L+1):
            wgt=0.5 if (k in (0,L)) else 1.0
            acc+=wgt*Hd[k]*math.cos(2*math.pi*fs[k]*(n-a)/FS1)
        h[n]=acc*df*2.0/(FS1/2)*win[n]
    dc=sum(h); h=[c*(1.0/CIC_DC)/dc for c in h]
    lim=1<<17; sc=1<<COEF_FRAC
    return [max(-lim, min(lim-1, int(round(c*sc)))) for c in h]

coef = design_comp_fir()

# deterministic LCG samples
_state = 0xCABBA9E5
def rnd(lo, hi):
    global _state
    _state = (_state*6364136223846793005+1442695040888963407) & ((1<<64)-1)
    return lo + (_state>>17) % (hi-lo+1)
samp = [[[rnd(-15000,15000) for _ in range(N_LANES)] for _ in range(N_SLOTS)]
        for _ in range(K_PACKETS)]

with open(f"{OUT}/cicch_samples.hex","w") as f:
    for p in range(K_PACKETS):
        for s in range(N_SLOTS):
            word=0
            for l in range(N_LANES): word|=(samp[p][s][l]&0xFFFF)<<(16*l)
            f.write(format(word,'x').zfill(32)+"\n")
with open(f"{OUT}/cicch_coefs.hex","w") as f:
    for j in range(HB_TAPS): f.write(to_hex(coef[j], COEF_W)+"\n")

enabled = [l for l in range(N_LANES) if (LANE_MASK>>l)&1]
SMAX,SMIN=(1<<(OUT_W-1))-1,-(1<<(OUT_W-1)); ROUND=1<<(COEF_FRAC-1)

# Stage 1: per channel CIC -> a stream of /5 outputs (signed int16)
cic_out = {}   # (l,s) -> list of CIC outputs (one per 5 input ticks)
for l in enabled:
    for s in range(N_SLOTS):
        integ=[0]*N_ORDER; cprev=[0]*N_ORDER; cnt=0; outs=[]
        for p in range(K_PACKETS):
            x=samp[p][s][l]&ACC_MASK; acc=x
            for i in range(N_ORDER): integ[i]=(integ[i]+acc)&ACC_MASK; acc=integ[i]
            cnt+=1
            if cnt==R_CIC:
                cnt=0; stage=integ[N_ORDER-1]
                for i in range(N_ORDER):
                    d=(stage-cprev[i])&ACC_MASK; cprev[i]=stage; stage=d
                y=sx(stage,ACC_W)>>GAIN_SHIFT
                y=SMAX if y>SMAX else SMIN if y<SMIN else y
                outs.append(y)
        cic_out[(l,s)]=outs

n_cic = K_PACKETS//R_CIC          # CIC outputs per channel
n_chain = n_cic//2                # halfband /2 outputs per channel

# Stage 2: halfband FIR /2 on each channel's CIC output stream
def hb_at(seq, idx):
    return seq[idx] if idx>=0 else 0
exp_val_fm, exp_chan_fm = [], []
chlist=[(l,s) for l in enabled for s in range(N_SLOTS)]
chain_out={}
for (l,s) in chlist:
    seq=cic_out[(l,s)]; outs=[]
    for p in range(n_cic):
        if (p%2)!=1: continue
        acc=sum(coef[j]*hb_at(seq,p-j) for j in range(HB_TAPS))
        r=(acc+ROUND)>>COEF_FRAC
        r=SMAX if r>SMAX else SMIN if r<SMIN else r
        outs.append(r)
    chain_out[(l,s)]=outs
# frame-major (the RTL emits lane-asc, slot-asc per output frame)
for fr in range(n_chain):
    for (l,s) in chlist:
        exp_val_fm.append(chain_out[(l,s)][fr])
        exp_chan_fm.append(l*N_SLOTS+s)

with open(f"{OUT}/cicch_exp_val.hex","w") as f:
    for v in exp_val_fm: f.write(to_hex(v, OUT_W)+"\n")
with open(f"{OUT}/cicch_exp_chan.hex","w") as f:
    for c in exp_chan_fm: f.write(to_hex(c, 16)+"\n")

print(f"chain /10: HB_TAPS={HB_TAPS} packets={K_PACKETS} cic_outs/ch={n_cic} "
      f"chain_outs/ch={n_chain} enabled={enabled} total_outs={len(exp_val_fm)} "
      f"coef[mid]={coef[HB_TAPS//2]}")
