import  action_group_controller as controller
#from ros_robot_controller_sdk import Board

#board = Board()
#controller = ActionGroupController(board)

# Action groups need to be saved in the ActionGroups folder under the current path (动作组需要保存在当前路径的ActionGroups下)
controller.runAction('five')  # Parameter is the name of the action group, without the suffix, passed as a string (参数为动作组的名称，不包含后缀，以字符形式传入)
