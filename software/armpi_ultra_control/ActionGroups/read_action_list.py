#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import threading
import sqlite3 as sql
import servo_controller as controller
import csv

runningAction = False
stopRunning = False
online_action_num = None
online_action_times = -1
update_ok = False
action_group_finish = True
setServoPulse = None

action_path = os.path.split(os.path.realpath(__file__))[0]

def stop_servo():
    for i in range(16):
        stopBusServo(i+1) 

def stop_action_group():
    global stopRunning, online_action_num, online_action_times, update_ok
    update_ok = False
    stopRunning = True
    online_action_num = None
    online_action_times = -1
    time.sleep(0.1)

def action_finish():
    global action_group_finish
    return action_group_finish  

def runAction(actNum):
    global runningAction
    global stopRunning
    global online_action_times
    if actNum is None:
        return
    
    actNum = os.path.join(action_path, 'ActionGroups', actNum + ".d6a")
    stopRunning = False
    if os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            
            #Create CSV file (创建 CSV 文件)
            csv_file_path = actNum.replace(".d6a", ".csv")  # Generate CSV file with the same name as the action group file（生成与动作组同名的 CSV 文件）
            with open(csv_file_path, mode='w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(["Servo ID", "Pulse Width", "Duration"])  # Write title row（写入标题行）
                
                while True:
                    act = cu.fetchone()
                    if stopRunning is True:
                        stopRunning = False                   
                        break
                    if act is not None:
                        #  Write to CSV file（写入到 CSV 文件）
                        for i in range(0, len(act)-2, 1):
                            if i < (len(act) - 3):
                                # Update servo pulse (更新伺服脉冲)
                                #controller.setServoPulse(i+1, act[2 + i], act[1])
                                # Write the current servo data to CSV (写入当前伺服数据到 CSV)
                                csv_writer.writerow([i+1, act[2 + i], act[1]])
                            else:
                                time.sleep(0.02)
                                #controller.setServoPulse(10, act[2 + i], act[1])
                
                        time.sleep(float(act[1])/1000.0)
                    else:   #  exit only after finishing the execution (运行完才退出)
                        break
            
            runningAction = False
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件")

def online_thread_run_acting():
    global online_action_times, online_action_num, update_ok, action_group_finish
    while True:
        if update_ok:
            if online_action_times == 0:
                # run infinitely(无限次运行)
                if action_group_finish:
                    action_group_finish = False
                runAction(online_action_num)                
            elif online_action_times > 0:
                # run for a certain number of times(有次数运行)
                if action_group_finish:
                    action_group_finish = False
                runAction(online_action_num)
                online_action_times -= 1    # After finishing the execution, enter the no-load state(运行完成后，进入空载)
                if online_action_times == 0:
                    online_action_times = -1
            else:
                # no-load(空载)
                if not action_group_finish:
                    action_group_finish = True
                time.sleep(0.001)
        else:
            if not action_group_finish:
                action_group_finish = True
            time.sleep(0.001)
            
def start_action_thread():
    th1 = threading.Thread(target=online_thread_run_acting)
    th1.setDaemon(True)  # Set as a background thread, which is 'True' by default(设置为后台线程，这里默认是True()
    th1.start()
    
def change_action_value(actNum, actTimes):
    global online_action_times, online_action_num, update_ok, stopRunning, action_group_finish
    
    if action_group_finish:
        online_action_times = actTimes
        online_action_num = actNum
        stopRunning = False
        update_ok = True
action_path = os.path.split(os.path.realpath(__file__))[0]

def export_action_groups_to_csv():
    #  Get all .d6a files in the specified directory (获取指定目录下所有 .d6a 文件)
    action_files = [f for f in os.listdir(action_path) if f.endswith('.d6a')]
    
    for act_file in action_files:
        act_file_path = os.path.join(action_path, act_file)
        #  Connect to the action group database (连接到动作组数据库)
        ag = sql.connect(act_file_path)
        cu = ag.cursor()
        cu.execute("select * from ActionGroup")
        
        # Create CSV files with the same name as the action group file (创建与动作组文件同名的 CSV 文件)
        csv_file_path = act_file_path.replace(".d6a", ".csv")
        with open(csv_file_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["Servo ID", "Pulse Width", "Duration"]) # Write the header row (写入标题行)
            
            while True:
                act = cu.fetchone()
                if act is not None:
                    for i in range(0, len(act)-2, 1):
                        csv_writer.writerow([i+1, act[2 + i], act[1]])   # Write servo data to CSV (写入伺服数据到 CSV)
                else:
                    break
        
       # Close the database connection (关闭数据库连接)
        cu.close()
        ag.close()
        print(f"Exported {act_file} to {csv_file_path}")

if __name__ == "__main__":
    export_action_groups_to_csv()