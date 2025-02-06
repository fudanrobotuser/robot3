# 定义初始值和目标值
initial_state = [105534, -22613, 2463803, -232823, -338509, -149367, -311665, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 初始位置

# 过渡点1
target_state_1 = [-7125, -53557, 2423322, -1321295, -1392415, 325981, 123518, 0, 0, 59423, 538, -93224, 133167, 1664830, 15591, 61676]

# 过渡点2
target_state_2 = [-11340, -1082800, 2388374, -1445238, -1392425, 333283, 123521, 0, 0, 59423, 538, -93224, 133167, 1664830, 15591, 61676]

# 左摆动点
left_swing = [-500000, -1000000, 2500000, -1200000, -1500000, 500000, 100000, 0, 0, 60000, 500, -90000, 140000, 1600000, 15000, 70000]

# 右摆动点
right_swing = [500000, 1000000, 2500000, 1200000, 1500000, -500000, -100000, 0, 0, 60000, 500, -90000, 140000, 1600000, 15000, 70000]

# 计算每个电机的渐变步骤（从初始位置到过渡点1）
num_steps = 200
gap_1 = [(target_state_1[i] - initial_state[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从过渡点1到过渡点2）
gap_2 = [(target_state_2[i] - target_state_1[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从过渡点2到左摆动点）
gap_3 = [(left_swing[i] - target_state_2[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从左摆动点到右摆动点）
gap_4 = [(right_swing[i] - left_swing[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从右摆动点到左摆动点）
gap_5 = [(left_swing[i] - right_swing[i]) / num_steps for i in range(16)]

# 计算每个电机的渐变步骤（从左摆动点回到过渡点2）
gap_6 = [(target_state_2[i] - left_swing[i]) / num_steps for i in range(16)]

# 打开文本文件（以写模式）
with open('salute_swing_6_times.csv', 'w') as file:
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
    for i in range(num_steps):
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
        for i in range(num_steps):
            row = [str(i + num_steps * 3 + 1 + swing * num_steps * 2)]  # 时间列继续递增
            for ii in range(16):
                data = left_swing[ii] + i * gap_4[ii]
                row.append(str(int(data)))  # 转换为整数并加入当前行
            file.write(",".join(row) + "\n")
        
        # 从右摆动点到左摆动点
        for i in range(num_steps):
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
