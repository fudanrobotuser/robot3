# 定义初始值和目标值
initial_state = [-10608,
 13396,
 3873827,
 -308951,
 -2219672,
 473027,
 -13427, 0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]  # 包含所有电机的初始值

# 过渡点1
target_state_1 = [332423,
 -38999,
 4105664,
 936370,
 -3097038,
 79070,
 336094,0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 过渡点2
target_state_2 = [332423,
 -38999,
 4105664,
 936370,
 -3097038,
 79070,
 336094,0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 左摆动点
left_swing = [1433328,
 -140796,
 4438366,
 1336499,
 -3739870,
 71024,
 318523,  0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 右摆动点
right_swing = [1463212,
 -140237,
 3675168,
 1336026,
 -3739885,
 71074,
 318575,  0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 计算每个电机的渐变步骤（从初始位置到过渡点1）
num_steps = 200
gap_1 = [(target_state_1[i] - initial_state[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从过渡点1到过渡点2）
gap_2 = [(target_state_2[i] - target_state_1[i]) / 10 for i in range(16)]

# 计算每个电机的渐变步骤（从过渡点2到左摆动点）
gap_3 = [(left_swing[i] - target_state_2[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从左摆动点到右摆动点）
gap_4 = [(right_swing[i] - left_swing[i]) / 100 for i in range(16)]

# 计算每个电机的渐变步骤（从右摆动点到左摆动点）
gap_5 = [(left_swing[i] - right_swing[i]) / 100 for i in range(16)]

# 计算每个电机的渐变步骤（从左摆动点回到过渡点2）
gap_6 = [(target_state_2[i] - left_swing[i]) / num_steps for i in range(16)]

gap_7 = [(target_state_1[i] - target_state_2[i]) / 10 for i in range(16)]

gap_8 = [(initial_state[i] - target_state_1[i]) / num_steps for i in range(16)]

# 打开文本文件（以写模式）
with open('waveHand.csv', 'w') as file:
    # 写入列标题
    file.write("Time,L1,L2,L3,L4,L5,L6,L7,R1,R2,R3,R4,R5,R6,R7,R8,R9\n")
    
    # 从初始位置到过渡点1
    for i in range(num_steps):
        row = [str(i + 1)]  # 时间列从 1 到 num_steps
        for ii in range(16):
            data = initial_state[ii] + i * gap_1[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")
    
    # 从过渡点1到过渡点2
    for i in range(10):
        row = [str(i + num_steps + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_1[ii] + i * gap_2[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")
    
    # 从过渡点2到左摆动点
    for i in range(num_steps):
        row = [str(i + num_steps * 2 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_2[ii] + i * gap_3[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")
    
    # 左右摆动6次：左->右->左->右->左->右
    for swing in range(3):  # 3次循环，共6次摆动
        # 从左摆动点到右摆动点
        for i in range(100):
            row = [str(i + num_steps * 3 + 1 + swing * num_steps * 2)]  # 时间列继续递增
            for ii in range(16):
                data = left_swing[ii] + i * gap_4[ii]
                row.append(str(int(data)))  # 转换为整数并加入当前行
            file.write(",".join(row) + "\n")
        
        # 从右摆动点到左摆动点
        for i in range(100):
            row = [str(i + num_steps * 4 + 1 + swing * num_steps * 2)]  # 时间列继续递增
            for ii in range(16):
                data = right_swing[ii] + i * gap_5[ii]
                row.append(str(int(data)))  # 转换为整数并加入当前行
            file.write(",".join(row) + "\n")
    
    # 从左摆动点回到过渡点2
    for i in range(num_steps):
        row = [str(i + num_steps * 7 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = left_swing[ii] + i * gap_6[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")

    #到过渡点1
    for i in range(10):
        row = [str(i + num_steps * 7 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_2[ii] + i * gap_7[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")

    for i in range(num_steps):
        row = [str(i + num_steps * 7 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_1[ii] + i * gap_8[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")