# 2025_ingbonbonusagi_robot
** 下載joint state publisher**

# 1. Update package lists
sudo apt update

# 2. Install joint_state_publisher (CLI)
sudo apt install ros-humble-joint-state-publisher

# 3. Install joint_state_publisher_gui (GUI)
sudo apt install ros-humble-joint-state-publisher-gui

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