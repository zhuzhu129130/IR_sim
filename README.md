# IR_sim
工程文件
20161229 add SSCA STCA,在parameters.txt中添加SSCA的参数，在stereoreconctruct.cpp中添加SSCA和STCA的匹配方式，便于在 上层程序中直接调用，使用1220的数据，得到结果还可以，不过时间有点长，不加后处理，就在头文件中注释不计算右图视差，大约0.5秒;若加上后处理，WM大约在4s左右，SG大约在11s左右。经测试，CEN+ST的组合还不错，其他的在做调整.


