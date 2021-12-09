import ctypes

def block_crc(byte_array):
	# MCRF4XX algorithm
    crcAccum = 0xffff
    for byte in byte_array:
        tmp = byte ^ bytearray(ctypes.c_uint16(crcAccum))[0]
        print(type(tmp))
        tmp ^= (tmp << 4)
        crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return ctypes.c_uint16(crcAccum)


if __name__ == '__main__':
	arr = "12345".encode("ascii")
	print(arr[0])
	print(block_crc(arr))
