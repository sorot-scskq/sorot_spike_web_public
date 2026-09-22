 
'''
SmartCarry.py
create data: 2023/09/14
Author     : Kaito Harada
'''


import cv2
import numpy as np

# 青サークル
B1 = 1
B2 = 2
B3 = 3
B4 = 4

# 赤サークル
R1 = 5
R2 = 6
R3 = 7
R4 = 8

# 黄サークル
Y1 = 9
Y2 = 10
Y3 = 11
Y4 = 12

# 緑サークル
G1 = 13
G2 = 14
G3 = 15
G4 = 16


UP         = 0
DOWN       = 1
RIGHT      = 2
LEFT       = 3
UP_RIGHT   = 4
UP_LEFT    = 5
DOWN_RIGHT = 6
DOWN_LEFT  = 7

'''
B1 - B4 - Y1 - Y4
'''

# サークル情報の初期化
CIRCLES = {
            # neighbors：移動可能サークル　　self_position：自身位置　　bottle：ボトルの有無
            B1: {'neighbors': {B2: LEFT,       B3: UP,        B4: UP_LEFT                                                                     }, 'self_position': 1, 'bottle': -1},
            B2: {'neighbors': {B1: RIGHT,      B3: UP_RIGHT,  B4: UP,       G1: LEFT,      G3: UP_LEFT                                        }, 'self_position': 0, 'bottle': -1},
            B3: {'neighbors': {B1: DOWN,       B2: DOWN_LEFT, B4: LEFT,     R1: UP,        R2: UP_LEFT                                        }, 'self_position': 0, 'bottle': -1},
            B4: {'neighbors': {B1: DOWN_RIGHT, B2: DOWN,      B3: RIGHT,    R1: UP_RIGHT,  R2: UP,        G1: DOWN_LEFT, G3: LEFT, Y1: UP_LEFT}, 'self_position': 0, 'bottle': -1},
            
            R1: {'neighbors': {B3: DOWN,       B4: DOWN_LEFT, R2: LEFT,     R3: UP,        R4: UP_LEFT                                        }, 'self_position': 0, 'bottle': -1},
            R2: {'neighbors': {B3: DOWN_RIGHT, B4: DOWN,      R1: RIGHT,    R3: UP_RIGHT,  R4: UP,        G3: DOWN_LEFT, Y1: LEFT, Y3: UP_LEFT}, 'self_position': 0, 'bottle': -1},
            R3: {'neighbors': {R1: DOWN,       R2: DOWN_LEFT, R4: LEFT                                                                        }, 'self_position': 0, 'bottle': -1},
            R4: {'neighbors': {R1: DOWN_RIGHT, R2: DOWN,      R3: RIGHT,    Y1: DOWN_LEFT, Y3: LEFT                                           }, 'self_position': 0, 'bottle': -1},
            
            Y1: {'neighbors': {B4: DOWN_RIGHT, R2:RIGHT,      R4: UP_RIGHT, G3: DOWN,      G4: DOWN_LEFT, Y2:LEFT,       Y3: UP,   Y4: UP_LEFT}, 'self_position': 0, 'bottle': -1},
            Y2: {'neighbors': {G3: DOWN_RIGHT, G4: DOWN,      Y1: RIGHT,    Y3: UP_RIGHT,  Y4: UP                                             }, 'self_position': 0, 'bottle': -1},
            Y3: {'neighbors': {R2: DOWN_RIGHT, R4: RIGHT,     Y1: DOWN,     Y2: DOWN_LEFT, Y4: LEFT                                           }, 'self_position': 0, 'bottle': -1},
            Y4: {'neighbors': {Y1: DOWN_RIGHT, Y2: DOWN,      Y3: RIGHT                                                                       }, 'self_position': 0, 'bottle': -1},
            
            G1: {'neighbors': {B2: RIGHT,      B4: UP_RIGHT,  G2: LEFT,     G3: UP,        G4: UP_LEFT                                        }, 'self_position': 0, 'bottle': -1},
            G2: {'neighbors': {G1: RIGHT,      G3: UP_RIGHT,  G4: UP                                                                          }, 'self_position': 0, 'bottle': -1},
            G3: {'neighbors': {B2: DOWN_RIGHT, B4: RIGHT,     R2: UP_RIGHT, G1: DOWN,      G2: DOWN_LEFT, G4: LEFT,      Y1: UP,   Y2: UP_LEFT}, 'self_position': 0, 'bottle': -1},
            G4: {'neighbors': {G1: DOWN_RIGHT, G2: DOWN,      G3: RIGHT,    Y1: UP_RIGHT,  Y2: UP                                             }, 'self_position': 0, 'bottle': -1}
        }



class SmartCarry:
    location = None
    circles  = None
    
    '''
    初期化
    '''
    def __init__(self):
        self.location = B1  # 初期位置を設定
        self.circles = CIRCLES.copy()
    
    '''
    自己位置管理
    '''
    # スマートキャリーコースでの自己位置を更新することで自身の位置と移動できる方向を管理する
    def LocationUpdate(self, new_location, bottle):
        if new_location in self.circles[self.location]['neighbors']:
            self.circles[self.location]['self_position'] = 0
            self.location = new_location
            self.circles[self.location]['self_position'] = 1
            self.circles[self.location]['bottle'] = bottle
            print(f"Moved to {new_location}")
        else:
            print(f"Cannot move to {new_location}. Not a neighboring circle.")
    
    def display_position(self):
        print("Current position:")
        for circle, info in self.circles.items():
            print(f"{circle}: self_position={info['self_position']}, bottle={info['bottle']}")
    

    '''
    ボトル判別
    '''
    # デブリ・デンジャボトルを画像の青・赤色の割合で判別する
    # （コース上のサークル色を読み込まないか不安）
    def BottleJudg(self, image):
        # 撮影した画像を取得
        # image_path = "image0803_13.jpg"
        # image      = cv2.imread(image_path)

        if image is None:
            print("Could not open or find the image")
            return -1

        # 画像の高さと幅を取得
        height, width, _ = image.shape
        print(f"Original image size: Width={width}, Height={height}")

        # 撮影した画像の上下左右から指定の割合を切り取る
        percentage_top    = 20
        percentage_bottom = 45
        percentage_right  = 30
        percentage_left   = 30

        top    = height * percentage_top            // 100
        bottom = height * (100 - percentage_bottom) // 100
        left   = width  * percentage_left           // 100
        right  = width  * (100 - percentage_right)  // 100

        # 切り取り範囲が正しいかを確認
        if top >= bottom or left >= right:
            print(f"Invalid cropping dimensions: top={top}, bottom={bottom}, left={left}, right={right}")
            return -1
        
        # 切り取った画像のサイズを確認
        roi           = image[top:bottom, left:right]
        cropped_image = roi

        print(f"Cropped image size: Width={cropped_image.shape[1]}, Height={cropped_image.shape[0]}")

        if cropped_image.size == 0:
            print("Cropped image is empty")
            return -1

        # 切り取った画像を表示
        cv2.imshow('Cropped Image', cropped_image)
        cv2.waitKey(0)  # キーが押されるまでウィンドウを保持
        cv2.destroyAllWindows()

        # 画像をHSVに変換
        hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        # 赤と青の範囲を定義
        lower_red1 = np.array([0,   50,  50])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 50,  50])
        upper_red2 = np.array([180, 255, 255])
        lower_blue = np.array([110, 50,  50])
        upper_blue = np.array([130, 255, 255])

        # 赤と青のマスクを作成
        red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # 赤と青のピクセルを数える
        red_count  = cv2.countNonZero(red_mask)
        blue_count = cv2.countNonZero(blue_mask)

        # 一定の値以下の場合 -1 を返すようにする
        threshold = 50  # 適切な閾値を設定してください
        if red_count <= threshold and blue_count <= threshold:
            return -1

        if red_count < blue_count:
            # デブリボトルの場合
            return 0
        else:
            # デンジャボトルの場合
            return 1
    
    def RunRoute(self):
        route = [B1, B4, Y1, Y4]
        
        for point in route:
            # Simulate bottle judgment with a placeholder image
            result = self.BottleJudg(np.zeros((100, 100, 3), dtype=np.uint8))
            
            if result == 1:  # デンジャボトルがある場合
                for neighbor, direction in self.circles[self.location]['neighbors'].items():
                    if self.circles[neighbor]['bottle'] == -1:  # ボトルがない場合
                        print(f"Detour to {neighbor} towards {direction}")
                        return direction  # 避ける方向を返す
            elif result == 0:  # デブリボトルの場合
                self.LocationUpdate(point, result)
                print("Continue along the basic route: 1")
                return 1  # 直進
            elif result == -1:  # ボトルの検出に失敗した場合
                print("Error in bottle judgement.")
                return -1
        
        return "Route complete"
        

if __name__ == "__main__":
    sc = SmartCarry()
    sc.display_position()
    direction = sc.RunRoute()
    print(f"Next move direction: {direction}")
    