# misora2_dt_client
 - デジタルツインへ報告を行うパッケージ
## ファイル
### src/dt_client_component.cpp 確認画面ノード
 - このノードでは、以下のデータおよびボタンを表示します：
    - qr_id：デコード内容 (String)
    - qr_image：画像 (Image) 
    - result_data：報告内容 (String) 
    - result_image：報告画像 (Image)
    - backボタン：報告を行わず、画面を閉じる
    - sendボタン：報告ノードへ起動トリガーを送信
#### 挙動の仕様
 - <font color="red">4つデータ(qr_id、qr_image、result_data、result_image)がすべて揃っていない状態でsendボタンを押しても、起動トリガーは送信されない。</font>
 - 起動トリガーの送信に成功した場合、ノード内で保持しているデータは初期化される。
 - backボタンを押した場合は、保持しているデータは初期化されず、そのまま残る。

### misora2_dt_client/client.py 報告ノード
#### 起動時のパラメータ
 - ノード起動時に以下のパラメータを取得する
    - host：RMS(Remote Management System)サーバーのアドレス
    - robot_id：ロボット個体を識別する任意のID
    - mac_id：ロボットが使用しているデバイスのMACアドレス　(自動取得)
    - mission：ローカル保存時のフォルダ名に使用される、ミッション識別子
#### 挙動の仕様
 - 起動時、パラメータに基づいて RMSサーバと通信を確立し、MACアドレスの登録を行う。
 - 確認画面ノードから 送信トリガー（send_trigger）を受信すると、保持中のデータを以下の処理をする：
    - ローカルに画像とCSVとして保存
    - RMSサーバへHTTPリクエストで送信
    - 送信完了後、データを初期化（ただし、トリガーが送られなかった場合は初期化しない）。

## 実行コード
~~~bash!
git clone git@github.com:WRS2025Pre-UoA/misora2_operation.git
cd [ワークスペース]
chmod +x src/misora2_dt_client/scripts/client_node.py
chmod +x src/misora2_dt_client/scripts/client_pos.py
colon build
source install/setup.bash
ros2 run misora2_dt_client dt_client_node # 確認画面ノード
ros2 run misora2_dt_client client_node.py # 報告ノード
ros2 run misora2_dt_client client_pos.py # 位置情報報告ノード
~~~