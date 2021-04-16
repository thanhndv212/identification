import io
import os
import binascii 
import base64
import struct
import pandas as pd

data = pd.read_csv('test_read_data.csv')

print(data['joint_1'].dtypes)
joint_1 = data['joint_1']
print(joint_1)
q = data.to_numpy()
print(q[:,0])


# count = 0
# with open('vide_1a6_100_0_1.bin','rb') as f:
		
	# for i in range(128):
	# 	record = f.read(16)
	# 	 = struct.unpack('' record)
	# 	print(I, A, B)



	# bin_data = f.read()
	# # base64_encoded = base64.b64encode(bin_data)
	# hex_data = binascii.hexlify(bin_data)
	# dec_data = int(hex_data)
	# print(dec_data)



	# while True:
	# 	count += 1
	# 	line = f.readline()
	# 	if not line:
	# 		break
	# 	print(count, line.strip())

