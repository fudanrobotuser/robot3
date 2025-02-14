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

# 第一个目标点
target_state_1 = [237700,
 -104802,
 4083369,
 -63275,
 -2558590,
 481817,
 175238, 0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 第二个目标点
target_state_2 = [1054619,
 -269468,
 4396382,
 -130464,
 -2143132,
 744007,
 -89960,0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 第三个目标点
target_state_3 = [1889454,
 -392200,
 5267981,
 -207601,
 -2120936,
 449627,
 -129722, 0, 0,  4575,
 -38361,
 -1466537,
 120834,
 1472626,
 88657,
 -61876]

# 计算每个电机的渐变步骤（到第一个目标点）
num_steps_1 = 200
gap_1 = [(target_state_1[i] - initial_state[i]) / num_steps_1 for i in range(16)]

# 计算每个电机的渐变步骤（到第二个目标点）
num_steps_2 = 200
gap_2 = [(target_state_2[i] - target_state_1[i]) / num_steps_2 for i in range(16)]

# 计算每个电机的渐变步骤（到第三个目标点）
num_steps_3 = 200
gap_3 = [(target_state_3[i] - target_state_2[i]) / num_steps_3 for i in range(16)]

# 计算每个电机的渐变步骤（从第三个目标点返回到第二个目标点）
gap_4 = [(target_state_2[i] - target_state_3[i]) / num_steps_3 for i in range(16)]

# 计算每个电机的渐变步骤（从第二个目标点返回到第一个目标点）
gap_5 = [(target_state_1[i] - target_state_2[i]) / num_steps_2 for i in range(16)]

# 计算每个电机的渐变步骤（从第一个目标点返回到初始位置）
gap_6 = [(initial_state[i] - target_state_1[i]) / num_steps_1 for i in range(16)]

# 打开文本文件（以写模式）
with open('pointTo.csv', 'w') as file:
    # 写入列标题
    file.write("Time,L1,L2,L3,L4,L5,L6,L7,R1,R2,R3,R4,R5,R6,R7,R8,R9\n")
    
    # 写入第一段渐变（从初始位置到目标点 1，150 步）
    for i in range(num_steps_1):
        row = [str(i + 1)]  # 时间列从 1 到 num_steps_1
        for ii in range(16):
            data = initial_state[ii] + i * gap_1[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔

    # 写入第二段渐变（从目标点 1 到目标点 2，300 步）
    for i in range(num_steps_2):
        row = [str(i + num_steps_1 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_1[ii] + i * gap_2[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔

    # 写入第三段渐变（从目标点 2 到目标点 3，300 步）
    for i in range(num_steps_3):
        row = [str(i + num_steps_1 + num_steps_2 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_2[ii] + i * gap_3[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔




with open('pointTo_.csv', 'w') as file:
    # 写入列标题
    file.write("Time,L1,L2,L3,L4,L5,L6,L7,R1,R2,R3,R4,R5,R6,R7,R8,R9\n")

    # 写入第四段渐变（从目标点 3 返回到目标点 2，300 步）
    for i in range(num_steps_3):
        row = [str(i + num_steps_1 + num_steps_2 + num_steps_3 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_3[ii] + i * gap_4[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔

    # 写入第五段渐变（从目标点 2 返回到目标点 1，300 步）
    for i in range(num_steps_2):
        row = [str(i + num_steps_1 + num_steps_2 + num_steps_3 + num_steps_3 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_2[ii] + i * gap_5[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔

    # 写入第六段渐变（从目标点 1 返回到初始位置，150 步）
    for i in range(num_steps_1):
        row = [str(i + num_steps_1 + num_steps_2 + num_steps_3 + num_steps_3 + num_steps_2 + 1)]  # 时间列继续递增
        for ii in range(16):
            data = target_state_1[ii] + i * gap_6[ii]
            row.append(str(int(data)))  # 转换为整数并加入当前行
        file.write(",".join(row) + "\n")  # 将当前行写入 CSV 文件，并用逗号分隔


