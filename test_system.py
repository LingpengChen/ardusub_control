#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub系统测试脚本
测试各个组件是否能正常工作
"""

import sys
import time
import subprocess
import importlib

def test_python_imports():
    """测试Python模块导入"""
    print("=== 测试Python模块导入 ===")
    
    required_modules = [
        'rospy',
        'pymavlink', 
        'matplotlib',
        'numpy',
        'tkinter',
        'threading',
        'time',
        'math'
    ]
    
    missing_modules = []
    
    for module in required_modules:
        try:
            importlib.import_module(module)
            print(f"✓ {module}")
        except ImportError as e:
            print(f"✗ {module} - 错误: {e}")
            missing_modules.append(module)
    
    if missing_modules:
        print(f"\n警告: 缺少以下模块: {missing_modules}")
        return False
    else:
        print("\n所有Python模块检查通过!")
        return True

def test_ros_environment():
    """测试ROS环境"""
    print("\n=== 测试ROS环境 ===")
    
    import os
    ros_distro = os.environ.get('ROS_DISTRO')
    
    if ros_distro:
        print(f"✓ ROS环境已设置: {ros_distro}")
        return True
    else:
        print("✗ ROS环境未设置")
        print("请运行: source /opt/ros/<distro>/setup.bash")
        return False

def test_file_existence():
    """测试文件是否存在"""
    print("\n=== 测试项目文件 ===")
    
    import os
    
    required_files = [
        'ros_keyboard_control_node.py',
        'ardusub_status_ui.py', 
        'ardusub_ros_interface_advanced.py',
        'start_ardusub_system.sh',
        'install_ros_dependencies.sh'
    ]
    
    missing_files = []
    
    for filename in required_files:
        if os.path.exists(filename):
            print(f"✓ {filename}")
        else:
            print(f"✗ {filename} - 文件不存在")
            missing_files.append(filename)
    
    if missing_files:
        print(f"\n警告: 缺少以下文件: {missing_files}")
        return False
    else:
        print("\n所有项目文件检查通过!")
        return True

def test_permissions():
    """测试文件权限"""
    print("\n=== 测试文件权限 ===")
    
    import os
    import stat
    
    executable_files = [
        'ros_keyboard_control_node.py',
        'ardusub_status_ui.py',
        'ardusub_ros_interface_advanced.py', 
        'start_ardusub_system.sh',
        'install_ros_dependencies.sh'
    ]
    
    permission_issues = []
    
    for filename in executable_files:
        if os.path.exists(filename):
            file_stat = os.stat(filename)
            if file_stat.st_mode & stat.S_IEXEC:
                print(f"✓ {filename} - 可执行")
            else:
                print(f"✗ {filename} - 不可执行")
                permission_issues.append(filename)
        else:
            permission_issues.append(filename)
    
    if permission_issues:
        print(f"\n警告: 以下文件权限有问题: {permission_issues}")
        print("运行以下命令修复: chmod +x *.py *.sh")
        return False
    else:
        print("\n所有文件权限检查通过!")
        return True

def test_ros_core():
    """测试ROS核心是否可用"""
    print("\n=== 测试ROS核心 ===")
    
    try:
        # 尝试导入rospy
        import rospy
        print("✓ rospy模块可用")
        
        # 检查roscore是否在运行
        result = subprocess.run(['pgrep', '-f', 'roscore'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ ROS核心正在运行")
            return True
        else:
            print("ℹ ROS核心未运行 (这是正常的，启动时会自动启动)")
            return True
            
    except Exception as e:
        print(f"✗ ROS测试失败: {e}")
        return False

def test_port_availability():
    """测试端口可用性"""
    print("\n=== 测试端口可用性 ===")
    
    import socket
    
    # 测试14551端口 (MAVLink)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', 14551))
        sock.close()
        print("✓ 端口14551可用")
        return True
    except socket.error as e:
        print(f"✗ 端口14551被占用: {e}")
        print("可能有其他MAVLink应用正在运行")
        return False

def run_syntax_check():
    """运行语法检查"""
    print("\n=== 语法检查 ===")
    
    python_files = [
        'ros_keyboard_control_node.py',
        'ardusub_status_ui.py',
        'ardusub_ros_interface_advanced.py'
    ]
    
    syntax_errors = []
    
    for filename in python_files:
        try:
            result = subprocess.run([sys.executable, '-m', 'py_compile', filename],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✓ {filename} - 语法正确")
            else:
                print(f"✗ {filename} - 语法错误:")
                print(result.stderr)
                syntax_errors.append(filename)
        except Exception as e:
            print(f"✗ {filename} - 检查失败: {e}")
            syntax_errors.append(filename)
    
    if syntax_errors:
        print(f"\n警告: 以下文件有语法错误: {syntax_errors}")
        return False
    else:  
        print("\n所有文件语法检查通过!")
        return True

def main():
    """主测试函数"""
    print("ArduSub ROS控制系统测试")
    print("=" * 50)
    
    test_results = []
    
    # 运行所有测试
    test_results.append(("Python模块导入", test_python_imports()))
    test_results.append(("ROS环境", test_ros_environment()))
    test_results.append(("项目文件", test_file_existence()))
    test_results.append(("文件权限", test_permissions()))
    test_results.append(("ROS核心", test_ros_core()))
    test_results.append(("端口可用性", test_port_availability()))
    test_results.append(("语法检查", run_syntax_check()))
    
    # 总结结果
    print("\n" + "=" * 50)
    print("测试结果总结:")
    print("=" * 50)
    
    passed_tests = 0
    total_tests = len(test_results)
    
    for test_name, result in test_results:
        status = "通过" if result else "失败"
        symbol = "✓" if result else "✗"
        print(f"{symbol} {test_name}: {status}")
        if result:
            passed_tests += 1
    
    print(f"\n总计: {passed_tests}/{total_tests} 项测试通过")
    
    if passed_tests == total_tests:
        print("\n🎉 所有测试通过! 系统可以正常使用!")
        print("\n下一步:")
        print("1. 连接ArduSub到电脑")
        print("2. 运行: ./start_ardusub_system.sh")
        return True
    else:
        print(f"\n⚠️  有 {total_tests - passed_tests} 项测试失败")
        print("\n建议:")
        print("1. 运行安装脚本: ./install_ros_dependencies.sh")
        print("2. 检查上述失败的测试项")
        print("3. 修复问题后重新运行测试")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
