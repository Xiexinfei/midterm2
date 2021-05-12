# midterm2
有四個RPC function，
（橘燈）gesture：在PC輸入讓mbed偵測手勢，判斷是哪個手勢後讓MQTT傳出來
angle：（此command由pyserial傳送）將mbed傳來的加速度存起來（因為超過五個gesture後會reset mbed）
（藍燈）analyse：在PC端輸入讓mbed分析加速度，
resewee：（此command由pyserial傳送）當gesture達到一定次數，reset mbed回到RPC loop

輸入/gesture/run 後進入詹側手勢模式，此function會開一個thread來偵測手勢，主程式會將手勢對應的mode印到uLCD上（0:ring, 1:slope, 2: straight），
並且在interruptin(button)按下之後，回傳加速度給PC，PC此時會將加速度存起來，接著reset mbed（以resewee function來reset），
再將加速度傳回mbed（以angle function來抓），reset mbed後PC可選擇輸入analyse萊分析角度
