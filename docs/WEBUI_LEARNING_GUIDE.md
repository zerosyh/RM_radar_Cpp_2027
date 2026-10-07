# 从零学会写雷达调试 Web UI

> 以本项目 `src/radar_web_ui/` 为完整案例。
> 目标读者:会写一点 C++ / Python,但没写过网页的人。
> 阅读方法:先看第 0~2 章建立地图,再按“后端 → 前端 → 自己动手”顺序精读。
> 代码行号以当前版本为准,以后代码改动时行号可能有小幅偏移,用 `grep -n "函数名" 文件` 定位。

---

## 0. 先建立整体认知(这一章最重要)

### 0.1 Web UI 的本质只有一句话

**浏览器(前端)向一个 HTTP 服务器(后端)要数据,后端把数据打包成网页或 JSON,浏览器负责画出来。**

我们这套 UI 没有 React、没有 Vue、没有 Node.js 打包工具,只用三样东西:

| 文件 | 语言 | 作用 |
| --- | --- | --- |
| `index.html` | HTML | 页面骨架:按钮、面板、画布放在哪里 |
| `styles.css` | CSS | 外观:颜色、尺寸、布局 |
| `app.js` | JavaScript | 大脑:要数据、更新页面、画地图、画曲线 |

后端 `web_ui_node.py` 是 Python:它一边订阅 ROS2 话题,一边开 HTTP 服务。

### 0.2 数据是怎么流动的

```text
C++ 雷达节点
  │  发布 ROS2 话题 (location / detect_result / map_view ...)
  ▼
Python 节点 web_ui_node.py
  │  1. 每个 ROS 消息进来,更新内存里的"最新状态"
  │  2. 内存状态打包成 JSON
  │  3. 通过 HTTP / SSE 发给浏览器
  ▼
浏览器 (app.js)
  │  1. fetch 或 EventSource 收到 JSON
  │  2. 把 JSON 存进 App.state
  │  3. requestAnimationFrame 每帧调 drawMap() 等函数
  ▼
<canvas> / <table> / <img> 显示出来
```

反向(浏览器改参数)的路径:

```text
用户点勾选框 / 改数字
  → app.js 用 fetch("PUT /api/debug") 发 JSON
  → Python 处理并保存
  → 下一次 SSE 快照带回新参数
  → 前端重新绘制
```

### 0.3 为什么选“原生三件套 + Python 标准库”

- 调试 UI 只在比赛机上用,不需要上万人访问,性能要求低。
- 不引入 npm / webpack,避免“构建比功能还难”的问题。
- Python `http.server` 是标准库,ROS2 官方环境自带 Python,部署零依赖。
- 以后想换 React / Vue,数据接口(JSON 结构)可以原样保留,前端重写即可。

### 0.4 推荐阅读顺序

1. 先跑起来:开 `web_ui_node` + `wave_mock_node`,浏览器打开 `http://127.0.0.1:8766`。
2. 边看网页边按 F12 打开开发者工具,点 Network / Console。
3. 读本文第 1 章补基础。
4. 读第 3 章后端精读(对着文件看)。
5. 读第 4 章前端精读(对着文件看)。
6. 按第 5 章自己从 0 写一个最小版。
7. 以后做新功能前查第 6、7 章。

---

## 1. 最少必要知识(零基础补课清单)

### 1.1 HTML:页面骨架

HTML 就是一棵标签树:

```html
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <title>标题</title>
    <link rel="stylesheet" href="/styles.css" />   <!-- 引入 CSS -->
  </head>
  <body>
    <header id="topBar">这里是顶栏</header>
    <section class="panel">
      <h2>地图</h2>
      <canvas id="mapCanvas"></canvas>              <!-- 画布,以后在上面画画 -->
      <button id="refreshBtn">刷新</button>
    </section>
    <script src="/app.js"></script>                 <!-- 引入 JS -->
  </body>
</html>
```

需要理解三个词:
- **标签**:`<section>`, `<button>` 等。
- **id**:页面唯一编号,JS 用 `document.getElementById("id")` 拿到它,本项目的 `$()` 就是它的简写。
- **class**:可重复的样式分类,CSS 用 `.class` 选择它。

本项目看:`web/index.html`。顶栏、三个 `<section class="page">`、地图页的左右布局、波页的四个 canvas、状态页的表格,全部是 HTML。

### 1.2 CSS:外观和布局

CSS 规则 = “选中谁 + 改成什么样”:

```css
.panel {                  /* 选中所有 class="panel" 的元素 */
  background: #131e2e;    /* 背景色 */
  border-radius: 10px;    /* 圆角 */
}
#mapCanvas {              /* 选中 id="mapCanvas" 的唯一元素 */
  position: absolute;     /* 绝对定位 */
  inset: 0;               /* 上下左右都贴住父容器 */
}
```

零基础只需要掌握 5 个概念,就能看懂本项目 90% 的 CSS:

| 概念 | 一句话解释 | 本项目例子 |
| --- | --- | --- |
| 盒模型 | 元素 = content + padding + border + margin | `.panel` 的 padding |
| 选择器 | `#id`、`.class`、`tag` | `#mapCanvas`、`.robot-row` |
| Flex | 一行/一列排列子元素 | `.app-header`、`.panel-head` |
| Grid | 表格式布局 | `.map-layout`、`.wave-grid`、`.status-grid` |
| 定位 | static/relative/absolute/fixed | `.map-tooltip` 悬浮框 |

本项目看:`web/styles.css`。建议从文件头 `:root`(颜色变量)开始,再看 `.map-layout` 和 `.wave-grid` 两段,布局就懂了。

### 1.3 JavaScript:大脑

JS 在浏览器里运行,零基础先掌握 7 个语法单元:

```javascript
// 1. 变量
let n = 3;                 // 可变
const App = {};            // 对象,本项目的全局状态

// 2. 函数
function add(a, b) { return a + b; }

// 3. 取页面元素
const canvas = document.getElementById("mapCanvas");

// 4. 修改元素内容
document.getElementById("summary").textContent = "运行中";

// 5. 事件:用户点击时执行
button.addEventListener("click", () => {
  console.log("被点击了");
});

// 6. 异步要数据(浏览器不卡住的网络请求)
const res = await fetch("/api/state");
const data = await res.json();     // JSON 文本 → JS 对象

// 7. 对象 / 数组
data.robots[0].current.x            // 取嵌套字段
data.robots.map(r => r.id)          // 数组遍历映射
```

Canvas 是另一块核心知识:HTML 的 `<canvas>` 是一块“画板”,JS 拿到 2D 画笔(`ctx`)后用坐标画线画圆:

```javascript
const ctx = canvas.getContext("2d");
ctx.fillStyle = "red";
ctx.beginPath();
ctx.arc(x, y, r, 0, Math.PI * 2);   // 圆心 x,y 半径 r
ctx.fill();
```

本项目看:`web/app.js:183 drawMap()` 全部是这种画圆画线的组合。

### 1.4 Python 后端:会 C++ 的人很好上手

后端需要 Python 的:
- `http.server.ThreadingHTTPServer` + `BaseHTTPRequestHandler`(处理 HTTP 请求)
- `threading.RLock`(多线程锁,概念和 C++ `std::mutex` 一样)
- `json.dumps / json.loads`(打包 / 解析 JSON)
- `rclpy.Node.create_subscription`(订阅 ROS2 话题,概念和 `create_subscription` in C++ 一样)

如果这些都面生,建议先用一天过一遍 Python 官方教程的 class / threading 两章。

### 1.5 推荐学习顺序(按天)

| 时间 | 内容 | 检验标准 |
| --- | --- | --- |
| 1~2 天 | HTML + CSS | 能手写一个带左右栏的页面 |
| 3~5 天 | JS 基础 + DOM + fetch | 能点按钮把一个数字显示到页面上 |
| 6~7 天 | Canvas 画图 + 定时器 | 能画一个圆并让圆每秒移动 |
| 8~9 天 | Python http.server + ROS2 rclpy 回调 | 能把 ROS 消息变成 HTTP JSON |
| 10 天 | 把第 5 章的 10 步独立做一遍 | 得到一个可运行的最小调试 UI |

---

## 2. 项目文件结构

```text
src/radar_web_ui/
├── package.xml                     # ROS2 包声明(告诉 colcon 怎么装)
├── setup.py / setup.cfg            # Python 包安装规则,注册可执行文件
├── resource/radar_web_ui           # ament 索引占位文件(内容为空)
├── launch/web_ui.launch.py         # ros2 launch 启动文件
├── radar_web_ui/
│   ├── __init__.py
│   ├── web_ui_node.py              # ★ 后端主文件(约 1300 行)
│   └── wave_mock_node.py           # 解析波模拟发布者(学习/联调用)
└── web/
    ├── index.html                  # ★ 页面骨架(约 280 行)
    ├── styles.css                  # ★ 外观(约 380 行)
    └── app.js                      # ★ 前端大脑(约 950 行)
```

配套文档:
- `src/radar_web_ui/README.md`:包使用说明。
- `docs/WEB_UI.md`:运行方法与解析波接口契约。
- 本文:教学分析。

---

## 3. 后端逐模块精读(web_ui_node.py)

### 3.1 程序总流程:main()

入口在 `web_ui_node.py:1271 main()`。它的执行顺序是:

```text
解析命令行参数 (--host --port --config)
  → rclpy.init()                     初始化 ROS2
  → RadarWebUiNode(...)              创建节点(同时完成所有订阅)
  → MultiThreadedExecutor            开 4 个线程跑 ROS 回调
  → RadarWebUiHttpServer(...)        创建 HTTP 服务
  → server.serve_forever()           在独立线程里无限处理浏览器请求
  → executor.spin()                  主线程处理 ROS 回调
  → Ctrl+C → 关服务器、关 executor、销毁节点
```

为什么 ROS 和 HTTP 要分开两个线程?因为 `executor.spin()` 会阻塞主线程,而 HTTP 服务器也要一直运行;两个互不阻塞才能同时工作。

### 3.2 节点初始化:RadarWebUiNode.__init__()

位置:`web_ui_node.py:347`。

依次做了四件事:

1. **读参数**:`declare_parameter` / `get_parameter` 拿到 host、port、config_path。ROS2 参数可用 `--ros-args -p port:=9000` 覆盖。
2. **找路径**:`_resolve_config_path()` 会在“命令行路径 → 当前目录 → ~/rm_lidar_2027/RM_radar_Cpp_2027”里找 `main_config.yaml`,这样从哪个目录启动都能找到配置。
3. **建状态容器**:
   ```python
   self.topic_stats = {}          # 每个话题的消息数/速率
   self.robot_states = {}         # 每辆车的最新位置 + 轨迹
   self.image_frames = {}         # 最新 JPEG 帧
   self.wave_store = WaveStore()  # 解析波数据
   self.debug_params = ...        # 页面可调参数
   ```
4. **订阅话题**:调用 `_setup_subscriptions()`(下一节)。

### 3.3 最重要的并发设计:一把锁 + 一个计数器

位置:`web_ui_node.py:354` 附近的 `self._lock` 和 `self._stream_condition`。

本程序同时有多个线程:
- ROS executor 线程:回调 `_location_cb` 等,不断**写**状态。
- HTTP 线程:每个浏览器请求一个线程,不断**读**状态。

Python 多线程读写共享数据必须加锁,否则会出现“读到一半被改”的脏数据。本项目统一用:

```python
self._lock = threading.RLock()          # 可重入锁
self._stream_condition = threading.Condition(self._lock)  # 条件变量
```

**写**的标准模板(所有 ROS 回调都这样):

```python
with self._lock:
    更新 self.xxx
    self._notify_locked()   # 见下
```

**读**的标准模板:

```python
with self._lock:
    data = 深拷贝或读取 self.xxx
```

`_notify_locked()`(`web_ui_node.py:515`)做两件事:

```python
self.generation += 1                  # 数据版本号 +1
self._stream_condition.notify_all()   # 唤醒所有等待的 SSE 连接
```

`generation` 是整个设计的关键:**它不是帧率,而是“数据变了多少次”的版本号**。SSE 连接只要发现 generation 变大了,就知道有新的状态可以推送;页面顶部的 `gen 12345` 就是它。

### 3.4 订阅与回调:_setup_subscriptions()

位置:`web_ui_node.py:554`。

```python
self.sub_location = self.create_subscription(
    Locations, "location", lambda msg: self._location_cb(msg, "location"), qos_loc)
```

语法和 C++ 版一一对应:
- `Locations` = 消息类型(`detect_result.msg.Locations`)
- `"location"` = 话题名
- lambda = 回调
- `qos_loc` = QoS(可靠、保留最近 10 条)

要特别注意 **QoS 必须和发布端兼容**。本项目的发布端:
- `radar.cpp` 的 `location` 是 reliable + depth 10 → 我们订阅 reliable。
- `detect.cpp` 的 `detect_result` 是 sensor_data QoS → 我们订阅 depth 5 reliable。
- 解析波 `/wave_analysis` 是未来接口,为了兼容 best_effort 发布者,我们**同时建了两个订阅**(reliable 和 best_effort),谁匹配就谁收到,这是 `web_ui_node.py:577` 附近的技巧。

回调分类:

| 回调 | 位置 | 处理 |
| --- | --- | --- |
| `_location_cb` | 592 | 把每辆车位置写入 `robot_states`,并追加轨迹点 |
| `_locations_list_cb` | 631 | 保存 sentry_targets / ai_nav 列表 |
| `_detect_cb` | 646 | 保存检测框列表 |
| `_ekf_diag_cb` | 668 | 保存 EKF 诊断槽 |
| `_pcd_cb` | 688 | 不存点云,只统计点数/帧率 |
| `_image_cb` | 696 | ROS Image → JPEG bytes |
| `_wave_ros_cb` | 766 | 解析 JSON 后交给 WaveStore |

学习重点看 `_location_cb` 的轨迹部分:

```python
entry["trail"].append({"t": now, "x": item["x"], "y": item["y"], "source": source})
```

它用一个有长度上限的 `deque(maxlen=120)` 保存最近 120 个位置,这就是地图上“尾巴”的数据来源。

`_topic_touch()`(`web_ui_node.py:525`)是所有话题统计的公共入口:记录消息数、时间戳,并用时间戳队列算 1 秒滑动速率。状态页的“速率Hz”就是它算的。

### 3.5 状态快照:snapshot()

位置:`web_ui_node.py:854`。这是后端唯一“出口数据”的打包函数。

浏览器每收到一次 SSE,拿到的 JSON 就是这里的返回值:

```json
{
  "ok": true,
  "generation": 123,
  "node": {"uptime": 12.3, "pid": 39, ...},
  "config": {...},          // 来自 main_config.yaml 的摘要
  "topics": [...],          // 话题速率表
  "robots": [...],          // 车辆位置 + 轨迹
  "sentry": [...],
  "ai_nav": [...],
  "detections": [...],
  "images": {...},          // 图像元信息(不含字节)
  "wave": {...},            // 解析波(最多带 2048 点)
  "logs": [...],
  "debug": {...}
}
```

两个设计细节值得学:
1. **图片字节不放进 JSON**。JPEG 可能几十 KB,放 JSON 会让 SSE 又慢又占内存;所以状态里只有宽高和序号,像素走独立的 `/stream/map_view` MJPEG 通道。
2. **解析波快照截断到 2048 点**。图表 2048 点足够,防止 10 个浏览器同时打开时推送爆炸。完整数据仍在 `WaveStore` 内存里。

### 3.6 HTTP 服务器:WebUiHandler

位置:`web_ui_node.py:986`。

`BaseHTTPRequestHandler` 是 Python 标准库提供的 HTTP 请求处理器,它自动调我们重写的方法:
- 浏览器 GET 请求 → `do_GET()`(`web_ui_node.py:1038`)
- 浏览器 POST → `do_POST()`(`web_ui_node.py:1090`)
- 浏览器 PUT → `do_PUT()`(`web_ui_node.py:1093`)

`do_GET` 的本质是“看路径,分派”:

```python
path = parsed.path
if path == "/api/state":     返回状态 JSON
elif path == "/api/events":  挂起并持续推 SSE
elif path == "/api/map.png": 返回地图图片
elif path == "/stream/map_view": MJPEG 视频流
else:                        当静态文件处理(优先 index.html)
```

**自己加新接口的模板**(比如以后想加 `/api/record`):

```python
# 在 do_GET 里加:
if path == "/api/record":
    self._send_json({"ok": True, "data": ...})
    return
```

POST/PUT 请求要先读 body,再解析 JSON(`web_ui_node.py:1023 _read_json_body`),所以前端用 `fetch(method:"PUT")` 改调试参数时,后端在 `_handle_mutation()` 里处理。

### 3.7 静态文件安全:_safe_static_path()

位置:`web_ui_node.py:1142`。

浏览器请求 `/index.html`、`/styles.css`、`/app.js`,服务器读文件返回。但**不能直接** `root + url_path` 读文件,否则攻击者可以请求 `/../../etc/passwd` 把服务器文件偷走,这叫路径穿越。

本项目用:

```python
root = os.path.abspath(web_root)
candidate = os.path.abspath(os.path.join(root, *relative.split("/")))
if os.path.commonpath([root, candidate]) != root:
    raise ValueError("非法路径")
```

意思是:拼出来的最终路径必须仍然在 web 目录里面,否则拒绝。

本项目还踩过一个坑:`symlink-install` 下 web 目录里的文件是软链接,`Path.resolve()` 会把软链接解析到源码目录,导致安全校验误判“非法路径”。所以这里用 `os.path.abspath()` 而不是 `Path.resolve()`——这也是为什么调试时浏览器报 `非法路径` 的原因。

### 3.8 SSE:服务器主动推送(_send_events)

位置:`web_ui_node.py:1191`。

普通 HTTP 是“浏览器问一次,服务器答一次”。但雷达数据每 0.1 秒变化,用轮询(fetch 每秒问)又慢又浪费。SSE(Server-Sent Events)是单向长连接:

```text
浏览器 EventSource("/api/events")
   └── HTTP 连接一直不挂断
        └── 服务器每有新数据,就写一行 "data: {json}\n\n"
```

后端代码骨架:

```python
# 1. 告诉浏览器:这是事件流
self.send_header("Content-Type", "text/event-stream")
# 2. 循环
while not shutdown:
    payload = self.app.snapshot()
    if payload["generation"] > 上次已发:
        self.wfile.write(b"data: " + json_bytes + b"\n\n")
        self.wfile.flush()          # 立刻冲出去,不能缓存
    else:
        self.wfile.write(b": keepalive\n\n")   # 心跳注释
        payload = self.app.wait_snapshot(...)  # 阻塞等新数据
```

必须理解四个细节:
1. **`flush()` 必须调**,否则操作系统缓冲会攒着不发送。
2. **必须发心跳**。有些代理/浏览器超时会关掉静默连接。
3. **限速 10Hz**。代码里 `last_send` 控制两次推送至少间隔 0.1 秒,防止 4 个话题 × 10Hz 叠加成 40Hz 推送。
4. 客户端断开时写 socket 会抛 `BrokenPipeError`,必须捕获并退出循环。

### 3.9 MJPEG 图像流:_send_mjpeg()

位置:`web_ui_node.py:1240`。

`<img src="/stream/map_view">` 请求的是“连续的 JPEG 流”。协议很简单:

```text
--rmradar-frame
Content-Type: image/jpeg
Content-Length: 7196

<JPEG 字节>
--rmradar-frame
...
```

浏览器 `<img>` 标签原生支持这种 multipart 流,所以前端什么都不用做,直接写 `<img src="...">` 就能显示视频。

JPEG 从哪来?ROS Image 回调 `_image_cb`(`web_ui_node.py:696`)里:
- `sensor_msgs/Image` 的 `data` 是一维字节数组
- 用 numpy reshape 成 H×W×(通道数)
- OpenCV `cv2.imencode(".jpg", mat)` 编码成 JPEG
- 存到 `self.image_frames`,MJPEG 循环每次拿最新帧

### 3.10 解析波仓库:WaveStore

位置:`web_ui_node.py:193`。

学习这个类可以掌握“接口兼容”的写法。未来解析波模块的数据字段谁也不知道,所以 `WaveStore.ingest()` 故意做得宽容:

- 字段别名表 `_SAMPLE_ALIASES`:`t` 和 `time` 都当 `t_ms`,`range` 和 `distance` 都当 `distance_m`。
- 任何数字自动 `float()` 转换。
- 每个样本自动加 `index`、`valid`。
- 返回 `{"ok": false, "error": ...}` 而不是抛异常,方便前端显示拒绝原因。

**关键设计:接口先定,实现后补。** 现在项目没有解析波,但页面、ROS 话题订阅、HTTP 端点、数据格式已经全部定好,以后解析波模块无论用 C++ 还是 Python,只要向 `/wave_analysis` 发 JSON 即可。

### 3.11 调试参数:_update_debug_params()

位置:`web_ui_node.py:919`。

前端 PUT `/api/debug`,后端会:
1. 查 `specs` 表:这个参数存在吗?是 bool 还是 int/float?范围多少?
2. 类型错误/超范围就拒绝或夹紧。
3. 更新 `self.debug_params`,写盘到 `~/.config/rm_radar_web_ui/debug_params.json`。
4. `generation += 1`,让所有浏览器立刻同步。

这个模式叫 **白名单 + 校验**:绝不能直接把前端传来的任意字段写进全局状态,否则一个拼写错误就会悄悄污染后端。

### 3.12 模拟发布者:wave_mock_node.py

位置:`src/radar_web_ui/radar_web_ui/wave_mock_node.py`。

它只有 100 行左右,是学习 rclpy 发布者最好的例子:

```python
self.publisher = self.create_publisher(String, "wave_analysis", 10)
self.create_timer(float(period), self.publish_tick)   # 每 period 秒调一次

def publish_tick(self):
    payload = {"source": "wave_mock_node", "seq": ..., "samples": [...]}
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)
    self.publisher.publish(msg)
```

注意它演示了一个重要约定:**复杂数据用 `std_msgs/String` 装 JSON**,这样不用为每个调试字段新建 ROS msg。

---

## 4. 前端逐模块精读(web/)

### 4.1 HTML:三页是怎么切换的

`web/index.html` 的结构:

```html
<header class="app-header"> ... 三个 tab 按钮 ... </header>

<section class="page active" id="page-map"> 第1页 </section>
<section class="page" id="page-wave" hidden> 第2页 </section>
<section class="page" id="page-status" hidden> 第3页 </section>

<script src="/app.js"></script>
```

页面切换没有重新加载网页,只是:
- 把三个 section 的 `hidden` 属性换来换去;
- 切换 tab 按钮的 active 样式。

看 `app.js:882 setupTabs()`:

```javascript
function activate(page) {
  App.activePage = page;
  pages.forEach(...)  // 谁 active 谁显示
  location.hash = "#" + page;   // 把当前页写进地址栏,刷新后仍留在本页
}
```

`location.hash` 是地址栏 `#` 后面的部分,所以三个页面的网址是 `/#map`、`/#wave`、`/#status`。

### 4.2 CSS:布局怎么搭

看 `styles.css` 三个关键规则:

```css
.map-layout {
  display: grid;
  grid-template-columns: minmax(0, 1fr) 400px;  /* 左自适应 + 右固定400px */
  height: 100%;
}
.wave-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;               /* 两列 */
}
.status-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
}
```

CSS 里 `fr` = 按比例分剩余空间。`grid` 比老式 float 布局容易得多:先想“页面要几列几行”,再写 `grid-template-columns` 即可。

### 4.3 全局状态:App 对象

`app.js:6`:

```javascript
const App = {
  state: null,        // 后端发来的最新 JSON(一切数据都在这)
  generation: 0,      // 本地见过的最大版本号
  selectedRobot: null,// 当前选中车辆 id
  debug: null,        // 调试参数
  mapImage: null,     // 地图底图 Image 对象
  activePage: "map",  // 当前页面
};
```

前端写法的核心纪律:**数据只进 `App.state`,页面只从 `App.state` 读**。这样不管数据来自 HTTP 还是 SSE,渲染逻辑都是同一套。

### 4.4 数据入口:先 fetch 一次,再用 SSE 持续更新

`app.js:73 loadStateOnce()` 和 `app.js:95 connectEvents()`。

启动时:

```javascript
// 1. 先 fetch 一次,页面立刻有数据(SSE 建立连接需要时间)
const data = await (await fetch("/api/state")).json();
applyState(data);

// 2. 再开 SSE,以后新数据自动来
const source = new EventSource(`/api/events?generation=${App.generation}`);
source.onmessage = (event) => applyState(JSON.parse(event.data));
```

注意 URL 里的 `generation` 参数:如果页面刷新时本地已有 gen 100,就告诉服务器“从 100 之后给我”。服务器会等新版本再推;没有新数据就发心跳。

`applyState()`(`app.js:120`)是所有数据的唯一入口:

```javascript
function applyState(state) {
  App.state = state;
  App.generation = Math.max(App.generation, state.generation);
  updateHeader();
  updateRobotList();
  renderRobotDetail();
}
```

以后加新数据,就在 `applyState` 里加一句“刷新对应面板”。

### 4.5 渲染循环:requestAnimationFrame

`app.js:916 loop()`:

```javascript
function loop(now) {
  if (App.activePage === "map" && now - App.lastRender > 50) {
    drawMap();                 // 每 50ms 重画一次地图(20fps)
  }
  requestAnimationFrame(loop); // 请求浏览器下一帧再调我
}
```

关键思想:**数据到达频率 ≠ 渲染频率**。SSE 可能 10~50Hz 到达,但屏幕 60fps 就够,而且只画当前页。`requestAnimationFrame` 是浏览器标准动画接口,比 `setInterval` 更省电、更平滑。

### 4.6 地图绘制:坐标变换是灵魂

`app.js:170 mapTransform()`:

```javascript
const scale = Math.min(w / f.width, h / f.height);  // 取较小缩放,保持比例
const dw = f.width * scale, dh = f.height * scale;
const ox = (w - dw) / 2, oy = (h - dh) / 2;         // 居中
return {
  px: (x) => ox + (x / f.width) * dw,
  py: (y) => oy + ((f.height - y) / f.height) * dh,  // 注意 y 翻转!
};
```

这里面有本项目的两个关键数学:

1. **等比缩放 + 居中**:如果直接把 28×15 米硬拉成任意画布宽高,横向和纵向比例会不一样,机器人会与地图上的场地线条错位。所以先算一个统一 `scale`,两侧留黑边。
2. **y 轴翻转**:赛场坐标 y 向上(0~15),而 canvas 的 y 向下。`f.height - y` 就是翻转。这与 C++ `display_panel.cpp` 里 `yy = map_px_h_ - y*100` 完全一致。

`drawMap()`(`app.js:183`)的绘制顺序(后画的盖住先画的):

```text
1. 清屏 + 画底图(或纯色 + 网格)
2. 坐标刻度
3. 轨迹线(每辆车一条,越新越不透明)
4. 检测投影小点
5. last_known 灰叉
6. 机器人圆点 + 编号 + 空中菱形
7. sentry 紫点 + AI-NAV 橙色菱形
```

**阅读技巧**:把 `drawMap` 拆成“一层一层”,每层只干一件事,新增一种显示元素就是加一层。

### 4.7 鼠标交互:命中检测

`app.js:370 nearestRobot()`:

```javascript
function nearestRobot(px, py) {
  let best = null, bestDist = 42;      // 42px 内才算点到
  for (const robot of App.state.robots) {
    const d = 像素距离(机器人, 鼠标);
    if (d < bestDist) { best = robot; bestDist = d; }
  }
  return best;
}
```

鼠标事件在 `app.js:384 setupMapInteraction()`:
- `mousemove`:找最近机器人,更新 tooltip 位置和文字。
- `click`:找到就设置 `App.selectedRobot`,然后刷新右侧详情。
- `mouseleave`:隐藏 tooltip。

这是所有“地图点击选中目标”类 UI 的标准做法:遍历 → 算距离 → 找最近。

### 4.8 右侧面板:列表 + 详情 + 调试参数

- 列表 `updateRobotList()`(`app.js:413`):把 `App.state.robots` 数组用 `.map()` 生成 HTML 字符串,一次塞进容器。点某一行设 `selectedRobot`。
- 详情 `renderRobotDetail()`(`app.js:457`):从数组里 `find` 出选中车辆,再用 `velocityOf()` 从最后两个轨迹点算速度:`v = (p2 - p1) / dt`。
- 调试参数 `buildDebugControls()`(`app.js:509`):按 `DEBUG_SPECS` 配置表自动生成每个开关/数字输入框,而不是手写 12 段 HTML。改值时:
  ```javascript
  fetch("/api/debug", { method: "PUT", body: JSON.stringify({[key]: value}) })
  ```
  这是“前端配置驱动生成控件”的常见写法,加参数只需往表里加一行。

### 4.9 解析波图表

通用折线图 `drawLineChart()`(`app.js:571`)是一个可以复用的函数:

```text
参数:canvas + points([{x,y},...]) + 颜色/轴标签
内部:量画布大小 → 算 min/max → 画网格 → 画轴刻度 → 连线
```

四个图共用它,只是传入不同数据:
- 时域:`{x: t_ms, y: amplitude}`
- 频谱:FFT 结果 `{x: Hz, y: magnitude}`
- 距离:`{x: distance_m, y: amplitude}`
- 角度:`{x: angle_deg, y: amplitude}`

FFT 在 `app.js:643 fftRadix2()`:标准 radix-2 迭代 FFT。零基础可以暂时只当黑盒用,但要理解:
- 输入是**等间隔采样**的振幅序列;
- 输出是每个频率分量的强度;
- 频率轴公式 `f[i] = i * sample_rate / n`,采样率来自 `meta.sample_rate_hz` 或按 `t_ms` 间隔估算。

`drawWavePage()`(`app.js:684`)还演示了“没有数据时怎么办”:检查 `wave.available`,为 false 就显示空态提示,而不是让图表报错。

### 4.10 状态页:全部是 DOM 表格渲染

`updateStatusPage()`(`app.js:815`)没有 canvas,全是:

```javascript
$("topicTable").innerHTML =
  "<tr><th>话题</th>...</tr>" +
  topicRows.map(t => `<tr><td>${t.name}</td>...</tr>`).join("");
```

学习点:
- **innerHTML 拼字符串**是最快上手的渲染方法;数据大/更新极快时可再学虚拟 DOM,但现在不需要。
- `escapeHtml()`(`app.js:61`)把 `<` `>` 转义,防止话题名等外部字符串把 HTML 结构弄坏(XSS 风险)。
- MJPEG 图片不需要 JS,`<img src="/stream/map_view">` 自己一直收流。

---

## 5. 从零写一遍:10 步做出一个最小调试 UI

建议单独建目录 `/tmp/myweb`(或项目里 `playground/`),一步步加代码。每步都要运行验证。

### 第 1 步:先让浏览器显示一句话

`server.py`:

```python
from http.server import BaseHTTPRequestHandler, HTTPServer

class H(BaseHTTPRequestHandler):
    def do_GET(self):
        body = b"<h1>hello radar</h1>"
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

HTTPServer(("0.0.0.0", 8000), H).serve_forever()
```

运行 `python3 server.py`,浏览器打开 `http://127.0.0.1:8000`。**验证:能看到 hello radar。**

学会:HTTP 响应的三件套 = 状态码 + header + body。

### 第 2 步:把 HTML 拆到独立文件

改 `do_GET`:

```python
from pathlib import Path

def do_GET(self):
    if self.path == "/":
        body = Path("index.html").read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        ...
```

新建 `index.html`:

```html
<!doctype html>
<html><head><meta charset="utf-8"><link rel="stylesheet" href="/style.css"></head>
<body>
  <div id="box">0</div>
  <button id="btn">加一</button>
  <script src="/app.js"></script>
</body></html>
```

再让服务器能返回 `/style.css` 和 `/app.js`(用 `mimetypes.guess_type` 判断类型)。**验证:样式生效。**

### 第 3 步:第一个 JSON 接口

`server.py` 加:

```python
import json, time
if self.path == "/api/state":
    body = json.dumps({"now": time.time(), "value": 42}).encode()
    self.send_response(200)
    self.send_header("Content-Type", "application/json")
    ...
```

`app.js`:

```javascript
async function refresh() {
  const data = await (await fetch("/api/state")).json();
  document.getElementById("box").textContent = data.value;
}
refresh();
```

**验证:页面显示 42。** 在终端里用 `curl http://127.0.0.1:8000/api/state` 先看 JSON,再让浏览器显示——这是后端调试基本顺序。

### 第 4 步:轮询让数字自己变

```javascript
setInterval(refresh, 1000);
```

**验证:数字每秒刷新。** 体会:轮询简单,但数据变化快时浪费请求,为下一课 SSE 做铺垫。

### 第 5 步:换成 SSE

后端加 `/api/events`:

```python
self.send_response(200)
self.send_header("Content-Type", "text/event-stream")
self.end_headers()
while True:
    self.wfile.write(f"data: {json.dumps({'value': int(time.time())})}\n\n".encode())
    self.wfile.flush()
    time.sleep(1)
```

前端:

```javascript
const es = new EventSource("/api/events");
es.onmessage = (e) => {
  document.getElementById("box").textContent = JSON.parse(e.data).value;
};
```

**验证:数字每秒跳,Network 面板里只有一个持续中的 events 请求。** 学会:SSE 是服务器推、浏览器收。

### 第 6 步:接上 ROS 回调(数据源变成真实雷达)

把“每秒造数字”换成 rclpy 订阅:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Backend(Node):
    def __init__(self):
        super().__init__("my_web")
        self.latest = {"value": 0}
        self.create_subscription(String, "/demo", self.cb, 10)

    def cb(self, msg):
        self.latest["value"] = msg.data
```

用一个线程跑 `rclpy.spin`,主线程或另一个线程跑 HTTP 服务器。**验证:用 `ros2 topic pub /demo std_msgs/msg/String "{data: hello}"` 后网页变化。** 学会:ROS 回调只是数据源,和网页无关,中间靠内存状态桥接。

### 第 7 步:画第一张地图(Canvas 坐标变换)

HTML 加 `<canvas id="map" width="800" height="600">`。JS:

```javascript
const W = 28, H = 15;                 // 场地 28x15 米
function px(x) { return x / W * canvas.width; }
function py(y) { return (H - y) / H * canvas.height; }  // y 翻转

ctx.arc(px(robot.x), py(robot.y), 10, 0, Math.PI * 2);
```

先手算验证:(0,0) 应在左下角,(28,15) 在右上角,(14,7.5) 在正中心。**验证:用 mock 发布三辆车,三个点位置符合直觉。**

### 第 8 步:加参数控件(POST/PUT 回路)

HTML 加 checkbox,JS 加:

```javascript
checkbox.addEventListener("change", async () => {
  await fetch("/api/debug", { method: "PUT",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({ show_grid: checkbox.checked }) });
});
```

后端解析 body → 存状态 → 下次快照带上。**验证:刷新页面后设置仍在(说明前后端状态闭环)。**

### 第 9 步:加图像流(MJPEG)

先不用 ROS,后端每 100ms 从 OpenCV 生成一张画着数字的 JPEG 发 multipart;HTML 里写 `<img src="/stream">`。再换成订阅 `sensor_msgs/Image`。**验证:页面显示“视频”。**

### 第 10 步:打包成 ROS2 ament_python 包

照抄 `radar_web_ui/package.xml`、`setup.py`、`setup.cfg`、`resource/` 四个文件(它们都很短),把代码放进 `your_pkg/` 和 `web/`,colcon build。**验证:`ros2 run your_pkg web_node` 可用。**

完成这 10 步,你就已经具备独立写本项目的全部基础。之后做的每个功能,本质都是这三件事:
1. 后端加一个订阅/一个接口;
2. 快照里加字段;
3. 前端加一个渲染函数。

---

## 6. 调试方法论(按顺序执行)

### 6.1 后端:永远先用 curl 验证接口

```bash
curl -s http://127.0.0.1:8766/api/state | python3 -m json.tool   # 看 JSON 结构
curl -s -X POST http://127.0.0.1:8766/api/wave/ingest \
  -H 'Content-Type: application/json' -d '{"samples":[{"amplitude":1}]}'
curl -s -X PUT http://127.0.0.1:8766/api/debug \
  -H 'Content-Type: application/json' -d '{"trail_enabled":false}'
```

浏览器报错之前,先确认后端接口本身是否正确。**网页问题有 80% 其实是接口返回的数据不对。**

### 6.2 前端:F12 开发者工具

| 面板 | 看什么 |
| --- | --- |
| Console | JS 报错(红色)。**先解决第一个报错**,后面的往往是连锁反应 |
| Network | 每个请求状态码;点 `/api/state` 看 Response;SSE/MJPEG 是否一直连接 |
| Elements | 修改后的 HTML 是否和预期一致(比如 id 拼错会直接看不到) |
| Sources | 给 app.js 打断点,一步步看 App.state 是什么 |

无头 Chrome 截图(不打开窗口快速检查布局):

```bash
google-chrome --headless=new --no-sandbox --user-data-dir=/tmp/chrome-test \
  --window-size=1600,1000 --timeout=8000 \
  --screenshot=/tmp/ui.png 'http://127.0.0.1:8766/#map'
```

### 6.3 没有真实雷达时:Mock 发布者

调试 UI 不需要真雷达。本项目的 `wave_mock_node.py` 就是例子。临时造 location 数据:

```python
# 只保留关键代码,在任意 ROS2 环境运行
import rclpy, math
from rclpy.node import Node
from detect_result.msg import Location, Locations

class Mock(Node):
    def __init__(self):
        super().__init__("mock_loc")
        self.pub = self.create_publisher(Locations, "location", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)
    def tick(self):
        self.t += 0.1
        loc = Location(); loc.id = 1; loc.label = "Red"
        loc.x = 5.0 + math.sin(self.t); loc.y = 6.0 + math.cos(self.t)
        msg = Locations(); msg.locs.append(loc); self.pub.publish(msg)
```

原则:**UI 和算法分开调**。UI 用假数据调通了,再接真话题,出问题就能确定是数据链路问题。

### 6.4 进程与端口

```bash
ss -ltnp | grep 8766          # 谁占了端口
ps -ef | grep web_ui_node     # 找进程
kill <PID>                    # 改完 Python 代码必须重启,进程不会热加载
```

本项目踩过的坑:旧进程没死,新进程绑定端口报 `Address already in use`。先 `ss -ltnp` 找 PID,再 kill,再启动。

### 6.5 常见报错速查表

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 浏览器 `404` | 路由没写 / 静态文件没安装 | 检查 `do_GET` 路径,重新 `colcon build` |
| `/` 返回 `非法路径` | 路径校验把 symlink 解析出去了 | 用 `os.path.abspath`,不要 `Path.resolve()` |
| SSE 一直“断开” | 客户端超时/服务器异常退出 | 看后端 traceback;curl `--max-time 3 /api/events` 先测 |
| 页面有数据但画不出来 | canvas 尺寸为 0(父容器还没布局) | 每次画前 `getBoundingClientRect()` 重新量尺寸 |
| 地图机器人和场地对不上 | 画布拉伸 / y 没翻转 | 检查 `mapTransform` 的 scale 和 `f.height-y` |
| `Math.min(...arr)` 报栈溢出 | 数组太大,展开运算符超限 | 用 `arr.reduce((a,b)=>Math.min(a,b))` |
| ROS 订阅收不到消息 | QoS 不兼容 / 话题名拼错 | `ros2 topic info <topic> -v` 对 QoS |
| JSON 报错 “not JSON serializable” | 数据里有 numpy 类型 | 后端统一 `_jsonable()` 转换 |
| 修改调试参数不生效 | 后端没 `notify` / 前端没重画 | 检查 `_notify_locked()` 和 `applyState` 链路 |

---

## 7. 关键设计原则与坑(面试级总结)

### 7.1 数据单向流动

ROS → 后端状态 → JSON → 前端 App.state → 渲染。任何一层都不要偷偷从别处读数据。这个原则让项目半年后还能看懂。

### 7.2 版本号 generation 是实时推送的核心

不要用“每秒推一次”想问题,而是“状态变了就 +1,SSE 看到 +1 就推”。这样数据 0Hz 时不浪费,50Hz 时有限速保护。

### 7.3 大二进制(图像)和结构化数据分开传

JSON 走 SSE,图片走 MJPEG。混在一起是新手最常见错误,页面会卡死。

### 7.4 前端坐标变换必须显式写成一个函数

所有“世界坐标 → 像素坐标”都从 `mapTransform` 走,禁止在画图代码里到处写 `x * 50`。改比例尺时只改一个函数。

### 7.5 后端并发:短临界区 + 拷贝出去

锁里只做最小操作;`snapshot()` 在锁内构造一个新 dict 返回,不要在锁外直接引用共享字典。Python 的 `RLock` 可以重入,所以 `snapshot` 内部再调带锁函数不会死锁。

### 7.6 前端字符串安全

任何来自数据/用户输入的文字插入 HTML 前都要 `escapeHtml`,否则话题名、日志文本可能破坏页面(XSS)。

### 7.7 接口契约先于实现

解析波页面就是在实践这个原则:先定义 URL、JSON schema、字段别名,再写模拟发布者,最后接真实模块。

### 7.8 性能三件套

- SSE 限速 10Hz;
- 快照截断(轨迹 120 点、波 2048 点、检测 60 条);
- 只渲染当前页(隐藏页不画)。

---

## 8. 独立练习(做完了就真的会了)

按难度排序:

1. **改颜色/改半径**:在 `styles.css` 和 `app.js` 里把红方颜色改成橙色,把机器人半径改成 20。目的:熟悉改样式和找常量。
2. **加一个“暂停刷新”按钮**:前端点按钮后停止 applyState 更新(数据仍接收,只不渲染);再点恢复。目的:掌握事件 + 状态。
3. **在地图上显示每辆车实时坐标文字**:在 `drawMap()` 机器人圆旁边画 `(x,y)`,并加一个 debug 开关。目的:练习 canvas 文本 + 调试参数全链路。
4. **给解析波页加“历史帧下拉框”**:后端已在 `wave.history` 保留历史摘要,前端加一个 `<select>`,选中后显示对应帧时间/点数。目的:练习 DOM + 后端数据利用。
5. **加 `/api/config/reload` 接口**:POST 后重新读 `main_config.yaml` 并返回新配置,前端状态页加“重新加载配置”按钮。目的:练习 POST 接口 + 前后端闭环。
6. **把 MJPEG 换成“每 5 秒 JPEG 快照”模式**:加一个 debug 开关,前端从 `<img src="/stream/map_view">` 换成定时 `fetch` base64 快照。目的:理解两种图像传输方式的取舍。
7. **订阅一个你自己的新 ROS 话题并显示**:在 `detect_result` 或 `std_msgs/String` 里选一个现有话题,或在 C++ 节点发布一个 JSON String,加进状态页话题表。目的:全链路实战。

---

## 9. 核心词汇表

| 词 | 一句话解释 |
| --- | --- |
| HTTP | 浏览器与服务器通信的协议;请求 + 响应 |
| GET / POST / PUT | 读 / 新增(或动作) / 修改 的请求方法 |
| JSON | 跨语言数据结构文本,Python dict ↔ JS object |
| REST API | 用 URL + HTTP 方法表达接口的风格 |
| SSE | 服务器向浏览器单向持续推送 |
| WebSocket | 双向实时通道(本项目没用,以后重交互可用) |
| MJPEG | 连续 JPEG 组成的“视频流”,`<img>` 可直接播放 |
| DOM | 浏览器把 HTML 解析成的对象树,JS 可修改 |
| EventSource | JS 里接收 SSE 的对象 |
| fetch | JS 里发 HTTP 请求的函数 |
| Canvas 2D | 用 JS 画点线圆的画板 |
| requestAnimationFrame | 浏览器下一帧回调,做动画渲染循环 |
| QoS | ROS2 消息传输质量设置(reliable / best_effort) |
| callback | 回调:数据到了自动执行的函数 |
| 临界区 / 锁 | 多线程读写共享变量的保护机制 |
| generation | 本项目“状态版本号”,驱动 SSE 推送 |
| snapshot | 后端把当前全部状态打包成 JSON 的动作 |

## 10. 推荐资料

- MDN 中文文档(HTML/CSS/JS 最权威):搜索 “MDN HTML 入门”、“MDN Canvas 教程”、“MDN Fetch”。
- Python 官方文档:`http.server`、`threading`、`json` 三个模块页。
- 本项目参考源:`shark-radar-system/radio` 的 `apps/match_rx/web/`(成熟的前端布局)与 `match_dashboard_node.py`(SSE 后端写法)。
- 本项目三个文件:`web_ui_node.py`、`app.js`、`index.html` —— 边读本文边看源码,效率最高。

> 学习建议:不要试图一次读懂 1300 行。先按第 5 章写最小版,再回来读大项目,你会发现大项目只是“最小版 + 更多回调 + 更多面板 + 更多防御”。
