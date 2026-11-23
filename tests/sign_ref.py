"""
sign_ref.py: host reference for the ecdsa and ed25519 test programs

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Plain Python ECDSA with the RFC 6979 nonce and Ed25519 (RFC 8032), used
to check the vectors in the test programs.

Usage:
  python3 tests/sign_ref.py
"""

import hashlib
import hmac

CURVES = {
    "P-256": dict(
        p=2**256 - 2**224 + 2**192 + 2**96 - 1,
        a=-3,
        b=0x5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B,
        n=0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551,
        gx=0x6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296,
        gy=0x4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5),
    "P-384": dict(
        p=2**384 - 2**128 - 2**96 + 2**32 - 1,
        a=-3,
        b=0xB3312FA7E23EE7E4988E056BE3F82D19181D9C6EFE8141120314088F5013875AC656398D8A2ED19D2A85C8EDD3EC2AEF,
        n=0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC7634D81F4372DDF581A0DB248B0A77AECEC196ACCC52973,
        gx=0xAA87CA22BE8B05378EB1C71EF320AD746E1D3B628BA79B9859F741E082542A385502F25DBF55296C3A545E3872760AB7,
        gy=0x3617DE4A96262C6F5D9E98BF9292DC29F8F41DBD289A147CE9DA3113B5F0B8C00A60B1CE1D7E819D7A431D7C90EA0E5F),
    "secp256k1": dict(
        p=2**256 - 2**32 - 977,
        a=0,
        b=7,
        n=0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEBAAEDCE6AF48A03BBFD25E8CD0364141,
        gx=0x79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798,
        gy=0x483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8),
}


def ec_add(c, P, Q):
    p = c["p"]
    if P is None:
        return Q
    if Q is None:
        return P
    if P[0] == Q[0] and (P[1] + Q[1]) % p == 0:
        return None
    if P == Q:
        lam = (3 * P[0] * P[0] + c["a"]) * pow(2 * P[1], -1, p) % p
    else:
        lam = (Q[1] - P[1]) * pow(Q[0] - P[0], -1, p) % p
    x = (lam * lam - P[0] - Q[0]) % p
    return (x, (lam * (P[0] - x) - P[1]) % p)


def ec_mul(c, k, P):
    R = None
    while k:
        if k & 1:
            R = ec_add(c, R, P)
        P = ec_add(c, P, P)
        k >>= 1
    return R


def bits2int(b, qlen):
    v = int.from_bytes(b, "big")
    blen = len(b) * 8
    return v >> (blen - qlen) if blen > qlen else v


def rfc6979_k(c, x, h1, hname):
    n = c["n"]
    qlen = n.bit_length()
    rlen = (qlen + 7) // 8
    hf = getattr(hashlib, hname)
    hlen = hf().digest_size
    xo = x.to_bytes(rlen, "big")
    ho = (bits2int(h1, qlen) % n).to_bytes(rlen, "big")
    V = b"\x01" * hlen
    K = b"\x00" * hlen
    K = hmac.new(K, V + b"\x00" + xo + ho, hf).digest()
    V = hmac.new(K, V, hf).digest()
    K = hmac.new(K, V + b"\x01" + xo + ho, hf).digest()
    V = hmac.new(K, V, hf).digest()
    while True:
        T = b""
        while len(T) * 8 < qlen:
            V = hmac.new(K, V, hf).digest()
            T += V
        k = bits2int(T, qlen)
        if 1 <= k < n:
            yield k
        K = hmac.new(K, V + b"\x00", hf).digest()
        V = hmac.new(K, V, hf).digest()


def ecdsa_sign(cname, x, msg, hname):
    c = CURVES[cname]
    n = c["n"]
    h1 = getattr(hashlib, hname)(msg).digest()
    e = bits2int(h1, n.bit_length()) % n
    for k in rfc6979_k(c, x, h1, hname):
        R = ec_mul(c, k, (c["gx"], c["gy"]))
        r = R[0] % n
        s = pow(k, -1, n) * (e + r * x) % n
        if r and s:
            return k, r, s


# Ed25519, RFC 8032 section 5.1
EP = 2**255 - 19
EL = 2**252 + 27742317777372353535851937790883648493
ED = -121665 * pow(121666, -1, EP) % EP
SQRTM1 = pow(2, (EP - 1) // 4, EP)


def ed_add(P, Q):
    (x1, y1), (x2, y2) = P, Q
    t = ED * x1 * x2 * y1 * y2 % EP
    return ((x1 * y2 + x2 * y1) * pow(1 + t, -1, EP) % EP,
            (y1 * y2 + x1 * x2) * pow(1 - t, -1, EP) % EP)


def ed_mul(k, P):
    R = (0, 1)
    while k:
        if k & 1:
            R = ed_add(R, P)
        P = ed_add(P, P)
        k >>= 1
    return R


def ed_recover_x(y, sign):
    if y >= EP:
        return None
    x2 = (y * y - 1) * pow(ED * y * y + 1, -1, EP) % EP
    if x2 == 0:
        return None if sign else 0
    x = pow(x2, (EP + 3) // 8, EP)
    if (x * x - x2) % EP:
        x = x * SQRTM1 % EP
    if (x * x - x2) % EP:
        return None
    if (x & 1) != sign:
        x = EP - x
    return x


EBY = 4 * pow(5, -1, EP) % EP
EB = (ed_recover_x(EBY, 0), EBY)


def ed_enc(P):
    return (P[1] | ((P[0] & 1) << 255)).to_bytes(32, "little")


def ed_dec(b):
    v = int.from_bytes(b, "little")
    y = v & ((1 << 255) - 1)
    x = ed_recover_x(y, v >> 255)
    return None if x is None else (x, y)


def ed_keys(seed):
    h = hashlib.sha512(seed).digest()
    a = int.from_bytes(h[:32], "little")
    a &= (1 << 254) - 8
    a |= 1 << 254
    return a, h[32:], ed_enc(ed_mul(a, EB))


def ed_sign(seed, msg):
    a, prefix, A = ed_keys(seed)
    r = int.from_bytes(hashlib.sha512(prefix + msg).digest(), "little") % EL
    R = ed_enc(ed_mul(r, EB))
    k = int.from_bytes(hashlib.sha512(R + A + msg).digest(), "little") % EL
    return R + ((r + k * a) % EL).to_bytes(32, "little")


def ed_verify(A, msg, sig):
    Ap, Rp = ed_dec(A), ed_dec(sig[:32])
    s = int.from_bytes(sig[32:], "little")
    if Ap is None or Rp is None or s >= EL:
        return False
    k = int.from_bytes(hashlib.sha512(sig[:32] + A + msg).digest(), "little") % EL
    return ed_mul(s, EB) == ed_add(Rp, ed_mul(k, Ap))


def words_le(v, n):
    return ", ".join("0x%08X" % ((v >> (32 * i)) & 0xFFFFFFFF) for i in range(n))


def main():
    # RFC 6979 A.2.5 and A.2.6 keys
    x256 = 0xC9AFA9D845BA75166B5C215767B1D6934E50C3DB36E89B127B8A622B120F6721
    x384 = int("6B9D3DAD2E1B8C1C05B19875B6659F4DE23C3B667BF297BA9AA47740787137D8"
               "96D5724E4C70A825F872C9EA60D2EDF5", 16)
    for cname, x in (("P-256", x256), ("P-384", x384), ("secp256k1", x256)):
        c = CURVES[cname]
        U = ec_mul(c, x, (c["gx"], c["gy"]))
        print(cname, "Ux %X" % U[0])
        print(cname, "Uy %X" % U[1])
        for hname in ("sha256", "sha512"):
            for m in (b"sample", b"test"):
                k, r, s = ecdsa_sign(cname, x, m, hname)
                print(cname, hname, m.decode(), "k %X" % k)
                print("   r %X" % r)
                print("   s %X" % s)
    k, r, s = ecdsa_sign("secp256k1", 1, b"Satoshi Nakamoto", "sha256")
    n = CURVES["secp256k1"]["n"]
    print("secp256k1 key 1 Satoshi Nakamoto k %X r %X s %X n minus s %X" % (k, r, s, n - s))

    print("sqrt(minus 1)", words_le(SQRTM1, 8))
    print("d", words_le(ED, 8))
    print("L", words_le(EL, 8))
    print("(p minus 5) / 8", words_le((EP - 5) // 8, 8))
    for seed, msg in (
            ("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60", ""),
            ("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb", "72"),
            ("c5aa8df43f9f837bedb7442f31dcb7b166d38535076f094b85ce3a2e0b4458f7", "af82")):
        sd, m = bytes.fromhex(seed), bytes.fromhex(msg)
        A = ed_keys(sd)[2]
        sig = ed_sign(sd, m)
        print("ed25519 pub", A.hex())
        print("ed25519 sig", sig.hex(), ed_verify(A, m, sig))

    try:
        from cryptography.hazmat.primitives.asymmetric import ec, ed25519
        from cryptography.hazmat.primitives import hashes
        from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature
        for cname, x, cc in (("P-256", x256, ec.SECP256R1()), ("P-384", x384, ec.SECP384R1()),
                             ("secp256k1", x256, ec.SECP256K1())):
            pk = ec.derive_private_key(x, cc).public_key()
            for hname, ho in (("sha256", hashes.SHA256()), ("sha512", hashes.SHA512())):
                for m in (b"sample", b"test"):
                    _, r, s = ecdsa_sign(cname, x, m, hname)
                    pk.verify(encode_dss_signature(r, s), m, ec.ECDSA(ho))
        sk = ed25519.Ed25519PrivateKey.from_private_bytes(bytes.fromhex(
            "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60"))
        assert sk.sign(b"") == ed_sign(bytes.fromhex(
            "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60"), b"")
        print("cryptography agrees")
    except ImportError:
        print("cryptography not installed, no second check")


if __name__ == "__main__":
    main()
