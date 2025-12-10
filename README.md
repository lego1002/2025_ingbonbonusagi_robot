# 2025_ingbonbonusagi_robot

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
