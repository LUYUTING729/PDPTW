#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
提取Gurobi优化器log文件中的关键统计信息
"""

import os
import re
import pandas as pd
from pathlib import Path


def extract_log_info(log_file):
    """
    从log文件中提取关键信息
    
    返回字典包含：
    - instance_name: 算例名称
    - rows: 模型行数
    - columns: 模型列数
    - nonzeros: 非零元素个数
    - binary_vars: 二进制变量个数
    - root_lp_value: Root node LP值
    - initial_solution: 初始解值
    - final_objective: 最终目标函数值
    - final_best_bound: 最终的最优上界
    - final_gap: 最终的GAP值(%)
    - time_limit: 时间限制(秒)
    - elapsed_time: 实际运行时间(秒)
    - explored_nodes: 探索节点数
    - simplex_iters: 单纯形迭代次数
    """
    
    info = {
        'log_file': os.path.basename(log_file),
        'instance_name': '',
        'rows': '',
        'columns': '',
        'nonzeros': '',
        'binary_vars': '',
        'root_lp_value': '',
        'initial_solution': '',
        'final_objective': '',
        'final_best_bound': '',
        'final_gap': '',
        'time_limit': '',
        'elapsed_time': '',
        'explored_nodes': '',
        'simplex_iters': '',
        'status': ''
    }
    
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # 提取算例名称
        match = re.search(r'Instance Name:\s+(\S+)', content)
        if match:
            info['instance_name'] = match.group(1)
        
        # 提取时间限制
        match = re.search(r'Time Limit:\s+(\d+)', content)
        if match:
            info['time_limit'] = match.group(1)
        
        # 提取模型规模
        match = re.search(r'Optimize a model with (\d+) rows, (\d+) columns and (\d+) nonzeros', content)
        if match:
            info['rows'] = match.group(1)
            info['columns'] = match.group(2)
            info['nonzeros'] = match.group(3)
        
        # 提取二进制变量个数
        match = re.search(r'Variable types:.*?(\d+) binary', content)
        if match:
            info['binary_vars'] = match.group(1)
        
        # 提取初始解 - 从"Loaded user MIP start with objective"
        match = re.search(r'Loaded user MIP start with objective ([\d.e+-]+)', content)
        if match:
            info['initial_solution'] = match.group(1)
        
        # 提取Root LP值 (Root relaxation)
        match = re.search(r'Root relaxation: objective ([\d.e+-]+)', content)
        if match:
            info['root_lp_value'] = match.group(1)
        
        # 提取最终结果 - 使用最后一个"Best objective"行来获取最终的目标值、上界和GAP
        # 使用findall来获取所有匹配，然后取最后一个
        matches = re.findall(r'Best objective ([\d.e+-]+), best bound ([\d.e+-]+), gap ([\d.]+)%', content)
        if matches:
            # 取最后一个匹配（最终结果）
            final_match = matches[-1]
            info['final_objective'] = final_match[0]
            info['final_best_bound'] = final_match[1]
            info['final_gap'] = final_match[2]
        
        # 提取运行时间和节点数
        match = re.search(r'Explored (\d+) nodes \((\d+) simplex iterations\) in ([\d.]+) seconds', content)
        if match:
            info['explored_nodes'] = match.group(1)
            info['simplex_iters'] = match.group(2)
            info['elapsed_time'] = match.group(3)
        
        # 提取状态
        if 'Time limit reached' in content:
            info['status'] = 'Time limit'
        elif 'Optimal objective' in content:
            info['status'] = 'Optimal'
        elif 'Optimize a model' in content and 'Explored' not in content:
            info['status'] = 'Incomplete'
        else:
            info['status'] = 'Unknown'
        
    except Exception as e:
        info['status'] = f'Error: {str(e)}'
    
    return info


def main():
    """主函数"""
    
    result_dir = '/home/yutinglu/amdahl/src/result'
    
    # 查找所有log文件
    log_files = sorted(Path(result_dir).glob('*.log'))
    
    if not log_files:
        print(f"在 {result_dir} 中未找到任何 .log 文件")
        return
    
    print(f"找到 {len(log_files)} 个log文件")
    print()
    
    # 提取所有log文件信息
    results = []
    for log_file in log_files:
        print(f"处理: {log_file.name}", end=' ... ')
        info = extract_log_info(str(log_file))
        results.append(info)
        print("完成")
    
    # 创建DataFrame
    df = pd.DataFrame(results)
    
    # 重新排列列顺序
    columns_order = [
        'instance_name', 'rows', 'columns', 'nonzeros', 'binary_vars',
        'root_lp_value', 'initial_solution', 'final_objective', 'final_best_bound', 'final_gap',
        'time_limit', 'elapsed_time', 'explored_nodes', 'simplex_iters', 'status', 'log_file'
    ]
    df = df[columns_order]
    
    # 转换数值列为浮点数（如果可能）
    numeric_columns = ['root_lp_value', 'initial_solution', 'final_objective', 'final_best_bound', 'final_gap', 'elapsed_time']
    for col in numeric_columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # 打印表格
    print("\n" + "="*180)
    print("优化求解统计表")
    print("="*180)
    
    # 创建格式化的显示版本
    df_display = df.copy()
    
    # 数值列格式化
    for col in ['root_lp_value', 'initial_solution', 'final_objective', 'final_best_bound']:
        df_display[col] = df_display[col].apply(
            lambda x: f"{x:,.2f}" if pd.notna(x) else "N/A"
        )
    
    df_display['final_gap'] = df_display['final_gap'].apply(
        lambda x: f"{x:.4f}%" if pd.notna(x) else "N/A"
    )
    
    df_display['elapsed_time'] = df_display['elapsed_time'].apply(
        lambda x: f"{x:,.2f}s" if pd.notna(x) else "N/A"
    )
    
    # 显示精简表格
    display_cols = ['instance_name', 'rows', 'columns', 'binary_vars', 'root_lp_value', 
                   'initial_solution', 'final_objective', 'final_best_bound', 'final_gap', 'elapsed_time', 'status']
    print(df_display[display_cols].to_string(index=False))
    
    print("\n" + "="*180)
    
    # 保存为CSV文件
    output_csv = os.path.join(result_dir, 'log_statistics.csv')
    df.to_csv(output_csv, index=False)
    print(f"\n✓ 统计数据已保存到: {output_csv}")
    
    # 保存为Excel文件（如果安装了openpyxl）
    try:
        output_excel = os.path.join(result_dir, 'log_statistics.xlsx')
        df.to_excel(output_excel, index=False, sheet_name='Statistics')
        print(f"✓ 统计数据已保存到: {output_excel}")
    except ImportError:
        print("提示: 未安装openpyxl，跳过Excel导出。可运行: pip install openpyxl")
    
    # 打印摘要统计
    print("\n" + "="*180)
    print("摘要统计")
    print("="*180)
    
    print(f"\n总算例数: {len(df)}")
    print(f"平均Root LP值: {df['root_lp_value'].mean():.2f}")
    print(f"平均最终GAP: {df['final_gap'].mean():.2f}%")
    print(f"平均运行时间: {pd.to_numeric(df['elapsed_time'], errors='coerce').mean():.2f}秒")
    
    # 按Gap排序显示
    print("\n" + "-"*180)
    print("按最终GAP从小到大排序:")
    print("-"*180)
    df_sorted = df.sort_values('final_gap', na_position='last').copy()
    df_sorted_display = df_sorted[['instance_name', 'final_gap', 'final_objective', 'final_best_bound', 'elapsed_time', 'status']].copy()
    df_sorted_display['final_gap'] = df_sorted_display['final_gap'].apply(
        lambda x: f"{x:.4f}%" if pd.notna(x) else "N/A"
    )
    df_sorted_display['final_objective'] = df_sorted_display['final_objective'].apply(
        lambda x: f"{x:,.2f}" if pd.notna(x) else "N/A"
    )
    df_sorted_display['final_best_bound'] = df_sorted_display['final_best_bound'].apply(
        lambda x: f"{x:,.2f}" if pd.notna(x) else "N/A"
    )
    df_sorted_display['elapsed_time'] = df_sorted_display['elapsed_time'].apply(
        lambda x: f"{x:,.2f}s" if pd.notna(x) and x != "" else "N/A"
    )
    print(df_sorted_display.to_string(index=False))


if __name__ == '__main__':
    main()
