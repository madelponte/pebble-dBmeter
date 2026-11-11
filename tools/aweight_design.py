"""Design a fixed-point A-weighting filter for 16 kHz, as 3 biquads, and check it.

LF part (zeros at DC, poles at f1,f1,f2,f3) by bilinear transform: those poles are far below
Nyquist so warping is negligible. The f4 double pole (12.2 kHz) is above Nyquist (8 kHz), so the
HF roll-off is fitted instead: a biquad fitted by least squares to the remaining error.
"""
import numpy as np

FS = 16000.0
Q = 30
f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217


def a_weight_db(f):
    f = np.asarray(f, dtype=float)
    ra = (f4**2 * f**4) / ((f**2 + f1**2) * np.sqrt((f**2 + f2**2) * (f**2 + f3**2)) * (f**2 + f4**2))
    return 20 * np.log10(ra) + 2.0


def hp1(fc):
    """bilinear of s/(s+w): returns (b, a) first order"""
    K = 2 * FS
    w = 2 * np.pi * fc
    g = K / (K + w)
    p = (K - w) / (K + w)
    return np.array([g, -g]), np.array([1.0, -p])


def mul(s1, s2):
    return np.convolve(s1[0], s2[0]), np.convolve(s1[1], s2[1])


def freqz(b, a, f):
    z = np.exp(-2j * np.pi * np.asarray(f) / FS)
    num = sum(bk * z**k for k, bk in enumerate(b))
    den = sum(ak * z**k for k, ak in enumerate(a))
    return num / den


s1 = mul(hp1(f1), hp1(f1))
s2 = mul(hp1(f2), hp1(f3))

fit_f = np.geomspace(500, 7500, 200)
lf = np.abs(freqz(*s1, fit_f) * freqz(*s2, fit_f))
target = 10 ** (a_weight_db(fit_f) / 20) / lf   # what the HF section must supply


def hf_section(params):
    # biquad with real zero pair and real pole pair: (1+b1 z^-1)(1+b2 z^-1)/((1+a1 z^-1)(1+a2 z^-1))
    b1, b2, a1, a2 = params
    b = np.convolve([1, b1], [1, b2])
    a = np.convolve([1, a1], [1, a2])
    return b, a


def cost(params):
    if any(abs(p) >= 0.98 for p in params):
        return 1e9
    b, a = hf_section(params)
    h = np.abs(freqz(b, a, fit_f))
    h1k = np.abs(freqz(b, a, [1000.0]))[0]
    t1k = (10 ** (a_weight_db([1000.0])[0] / 20)) / np.abs(freqz(*s1, [1000.0]) * freqz(*s2, [1000.0]))[0]
    h = h * (t1k / h1k)
    err = 20 * np.log10(h / target)
    return float(np.sum(err**2))


# Nelder-Mead (numpy only)
def nelder_mead(f, x0, step=0.2, iters=4000):
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + step * np.eye(n)[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = np.argsort(vals)
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        c = np.mean(pts[:-1], axis=0)
        xr = c + (c - pts[-1]); fr = f(xr)
        if fr < vals[0]:
            xe = c + 2 * (c - pts[-1]); fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c); fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0] + 0.5 * (p - pts[0]) for p in pts]
                vals = [f(p) for p in pts]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


best = None
for x0 in ([0.5, 0.2, 0.3, -0.2], [0.9, 0.5, 0.6, 0.1], [0.2, -0.3, -0.1, -0.5], [0.8, 0.8, 0.5, 0.5]):
    p, v = nelder_mead(cost, x0)
    if best is None or v < best[1]:
        best = (p, v)
params = best[0]
s3 = hf_section(params)

# Normalise overall gain to 0 dB at 1 kHz (A-weighting is 0.0 dB there by definition).
g = 1.0 / np.abs(freqz(*s1, [1000.0]) * freqz(*s2, [1000.0]) * freqz(*s3, [1000.0]))[0]
g *= 10 ** (a_weight_db([1000.0])[0] / 20)
# Put the gain into section 3's numerator.
s3 = (s3[0] * g, s3[1])

# Sections 1 and 2 have a double zero at DC, so their numerators are b0 * (1 - 2z^-1 + z^-2).
# Move both b0 gains into section 3: sections 1 and 2 then need no numerator multiplies at all,
# and their DC zeros are exact instead of quantised.
for s in (s1, s2):
    assert np.allclose(s[0] / s[0][0], [1, -2, 1])
s3 = (s3[0] * s1[0][0] * s2[0][0], s3[1])
s1 = (np.array([1.0, -2.0, 1.0]), s1[1])
s2 = (np.array([1.0, -2.0, 1.0]), s2[1])

sections = [s1, s2, s3]


def quant(x):
    return int(round(x * (1 << Q)))


# Feedback coefficients are stored negated (-a1, -a2) so the C code only ever accumulates.
qsec = []
for b, a in sections:
    qsec.append(([quant(v) for v in b], [-quant(v) for v in a[1:]]))

# Fixed-point simulation of the C implementation (bit-exact with src/c/aweight.c).
IN_SHIFT = 12
OUT_SHIFT = IN_SHIFT - 6
I32 = (-(1 << 31), 1 << 31)


def fits32(*vals):
    for v in vals:
        assert I32[0] <= v < I32[1], "overflow"


def run_fixed(x16):
    # Direct form I, cascaded: each section's output history is the next section's input history,
    # so the state is just the last two values at each of the 4 nodes.
    z = [[0, 0] for _ in range(4)]
    out = []
    half = 1 << (Q - 1)
    (_, na_1), (_, na_2), (b3, na_3) = qsec
    for s in x16:
        x = int(s) << IN_SHIFT
        u = x - 2 * z[0][0] + z[0][1]
        fits32(u, 2 * z[0][0])
        u += (half + na_1[0] * z[1][0] + na_1[1] * z[1][1]) >> Q
        fits32(u)
        v = u - 2 * z[1][0] + z[1][1]
        fits32(v, 2 * z[1][0])
        v += (half + na_2[0] * z[2][0] + na_2[1] * z[2][1]) >> Q
        fits32(v)
        y = (half + b3[0] * v + b3[1] * z[2][0] + b3[2] * z[2][1]
             + na_3[0] * z[3][0] + na_3[1] * z[3][1]) >> Q
        fits32(y)
        for node, val in zip(z, (x, u, v, y)):
            node[1], node[0] = node[0], val
        out.append(y >> OUT_SHIFT)
    return np.array(out, dtype=float) / (1 << (IN_SHIFT - OUT_SHIFT))


# IEC 61672-1 class 2 tolerances (approx) at third-octave centres
iec = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800,
       1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 7000]
print("  f(Hz)   A(dB)  float(dB)  fixed(dB)   err")
worst = 0
for f in iec:
    ideal = a_weight_db([f])[0]
    h = np.abs(np.prod([freqz(b, a, [f])[0] for b, a in sections]))
    fl = 20 * np.log10(h)
    n = int(FS * 1.0)
    t = np.arange(n) / FS
    amp = 16000
    x = np.round(amp * np.sin(2 * np.pi * f * t))
    y = run_fixed(x)[n // 2:]
    fx = 20 * np.log10(np.sqrt(np.mean(y**2)) / (amp / np.sqrt(2)))
    err = fx - ideal
    if f >= 20:
        worst = max(worst, abs(err))
    print(f"{f:7.1f} {ideal:7.2f} {fl:9.2f} {fx:10.2f} {err:6.2f}")
print("worst |err| 20 Hz..7 kHz:", round(worst, 2), "dB")

# full-scale overflow check with a worst-case-ish signal
rng = np.random.default_rng(1)
run_fixed(np.clip(rng.normal(0, 20000, 16000), -32768, 32767))
run_fixed(np.where(np.arange(16000) % 2, 32767, -32768))
run_fixed(np.where(np.arange(16000) // 400 % 2, 32767, -32768))   # full-scale 20 Hz square: steps
peak = fit_f[np.argmax(a_weight_db(fit_f))]
run_fixed(np.round(32767 * np.sin(2 * np.pi * peak * np.arange(16000) / FS)))
print("overflow checks passed")

(_, na_1), (_, na_2), (b3, na_3) = qsec
print("\n// C coefficients, Q%d (feedback negated)" % Q)
print("#define S1_NA1 %d\n#define S1_NA2 %d" % tuple(na_1))
print("#define S2_NA1 %d\n#define S2_NA2 %d" % tuple(na_2))
print("#define S3_B0 %d\n#define S3_B1 %d\n#define S3_B2 %d" % tuple(b3))
print("#define S3_NA1 %d\n#define S3_NA2 %d" % tuple(na_3))
