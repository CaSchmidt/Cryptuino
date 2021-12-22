import scipy.io as io

var = {}
var['struct1'] = {}
var['struct1']['a']=16
var['struct1']['b']=32

io.savemat('packed_field_name_uncompressed_le.mat', var, do_compression=False)
io.savemat('packed_field_name_compressed_le.mat', var, do_compression=True)
