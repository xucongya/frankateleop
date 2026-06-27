import grpc
import sys
import time

def test_connection(ip, port):
    target = f"{ip}:{port}"
    print(f"尝试连接: {target}")
    
    # 尝试不同的通道选项
    options = [
        ('grpc.max_receive_message_length', 100 * 1024 * 1024),
        ('grpc.max_send_message_length', 100 * 1024 * 1024),
        ('grpc.keepalive_time_ms', 10000),
        ('grpc.keepalive_timeout_ms', 5000),
    ]
    
    channel = grpc.insecure_channel(target, options=options)
    
    # 等待连接就绪
    try:
        grpc.channel_ready_future(channel).result(timeout=10)
        print(f"✅ 成功连接到 {target}")
        return channel
    except grpc.FutureTimeoutError:
        print(f"❌ 连接超时: {target}")
        return None
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return None

if __name__ == "__main__":
    # 测试不同端口
    for port in [50051, 50052]:
        test_connection("192.168.1.100", port)
