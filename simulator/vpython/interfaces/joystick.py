import hid

# StampFly Controller USB HID settings
# StampFlyコントローラのUSB HID設定
VENDOR_ID = 0x303a   # Espressif VID
PRODUCT_ID = 0x8001  # StampFly Controller PID (sdkconfig.defaults)


class Joystick:
    def __init__(self, vendor_id=VENDOR_ID, product_id=PRODUCT_ID):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.device = None

    def open(self):
        try:
            # hidapi package: hid.device() (lowercase), then open(vid, pid)
            # hidapi パッケージ: hid.device()（小文字）で生成し、open(vid, pid)で接続
            self.device = hid.device()
            self.device.open(self.vendor_id, self.product_id)
            manufacturer = self.device.get_manufacturer_string()
            product = self.device.get_product_string()
            print("デバイスをオープンしました:", manufacturer, product)

            # 非ブロッキングモードに設定
            # Set non-blocking mode
            self.device.set_nonblocking(1)
        except Exception as e:
            print("エラー:", e)
            self.device = None

    def close(self):
        try:
            if self.device is not None:
                self.device.close()
        except Exception:
            print("デバイスをクローズ失敗")

    def read(self):
        if self.device is None:
            return None
        data = self.device.read(8)  # 1回の読み込みで最大8バイト取得
        if data:
            return data
        return None

    def write(self, data):
        if self.device is None:
            return
        self.device.write(data)

    def __del__(self):
        self.close()

    def list_hid_devices(self):
        """接続されているHIDデバイスの情報を列挙する
        List connected HID devices"""
        print("=== 接続されているHIDデバイス一覧 ===")
        for d in hid.enumerate():
            info = {
                'vendor_id': hex(d['vendor_id']),
                'product_id': hex(d['product_id']),
                'manufacturer': d.get('manufacturer_string'),
                'product': d.get('product_string')
            }
            print(info)
        print("====================================\n")

