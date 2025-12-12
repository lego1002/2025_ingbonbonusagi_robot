# 2025_ingbonbonusagi_robot
** 下載joint state publisher**
你指的是那個\*\*「有滑桿 (Slider) 可以拉動關節」\*\*的視窗工具。

你當時遇到的狀況是 `apt` 找不到套件，我們後來發現是因為你的軟體源 (Repository) 清單裡沒有加入 ROS 官方的來源。

以下是你成功的安裝步驟回顧：

### 1\. 核心安裝指令

在 Ubuntu 22.04 (ROS 2 Humble) 上，這個套件的名稱叫做 `ros-humble-joint-state-publisher-gui`。

指令是：

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
```

### 2\. 當時發生的插曲 (找不到套件)

你一開始執行上面那行失敗了 (顯示 `E: 找不到套件...`)。
我們後來做了這三步才修好：

1.  **加 Key**：`sudo curl ... ros.key ...`
2.  **加 Repo**：把 `http://packages.ros.org/ros2/ubuntu` 加到 sources list。
3.  **再 Update**：`sudo apt update` 之後才成功安裝。

### 3\. 如何使用它？

安裝完後，通常有兩種用法：

  * **單獨執行 (測試用)**：
    ```bash
    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    ```
  * **寫在 Launch 檔裡 (你現在的做法)**：
    你在 `view_robot.launch.py` 裡面已經加入了這個 Node：
    ```python
    Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    ```

**小提醒：**
現在你已經寫了 `ik_controller.py` (那個 Python IK 程式)，**那個程式會發布 `/joint_states`**。
所以當你執行 IK 程式時，記得要把 Launch 檔裡的 `joint_state_publisher_gui` **註解掉或關掉**，不然兩個程式同時搶著發布關節角度，機器人會在 Rviz 裡瘋狂抖動（鬼畜）。
**about using matlab simulation**
1. 先下載 peter corke的robot toolbox
   link: https://petercorke.com/toolboxes/robotics-toolbox/

2. 然後點按下載來的RTB檔案，matlab會自己幫你裝好add-ons

3. 在matlab指令區打startup_rvc來啟動toolbox

4. 如果不能用可能是命名重疊到，請使用以下解法:
   在 MATLAB 上方選單欄，點選 「Home」 分頁。

找到 「Set Path」 (設定路徑) 按鈕並點開。

在跳出的視窗清單中，往下拉，找到所有跟 Robotics Toolbox for MATLAB 相關的資料夾（通常是反白的，或者路徑很長在 AppData... 那邊）。

技巧：你可以按住 Shift 或 Ctrl 一次選取多個相關資料夾。

選好後，點擊視窗右側的 「Move to Top」 (移至頂層) 按鈕。

這一步就是告訴 MATLAB：以後遇到 rotx，先用 Peter Corke 的，不要用官方的。

點擊 「Save」，然後 「Close」。

**about the python file of webpage (folder: robot_web_interface)**
由於先前在電腦本地端使用 virtual environment 管理 flask 等套件，在此以 .gitignore檔忽略　git 對所有虛擬環境的追蹤。
若之後要在其他裝置執行網頁程式，請先獨立建立本地端虛擬機並安裝相關套件(pip3 install flask, flask_cors, request)，或是直接安裝於 python 版本中。