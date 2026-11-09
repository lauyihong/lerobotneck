// LeRobotNeck UI strings, English first. tr() also resolves route-file text given as {en, zh}.
const DICT = {
  en: {
    'nav.home': 'Home', 'nav.play': 'Simulator', 'nav.neck': 'How it’s made', 'nav.docs': 'Docs', 'nav.github': 'GitHub',
    'lang.other': '中文',
    'loading': 'Loading the 3D model…', 'loading.pct': 'Loading the 3D model… {p}%', 'loading.fail': 'Could not load the 3D model: {e}',
    'home.kicker': 'Open-source robot neck',
    'home.title': 'LeRobotNeck',
    'home.lead': 'One standard SO101 arm + two new 3D-printed parts = a 6-DoF neck that carries a RealSense D435i.',
    'home.stat.dof': 'degrees of freedom', 'home.stat.parts': 'new printed parts', 'home.stat.cam': 'depth camera',
    'home.cta.play': 'Try the simulator', 'home.cta.neck': 'See how it’s made',
    'home.hint': 'Drag to look around · click the neck to take it apart',
    'home.tip': 'Click to take it apart',
    'footer': 'Community project built on LeRobot, SO-ARM100 and XLeRobot (Apache-2.0). Not affiliated with Hugging Face. Simulation only.',
    'play.title': 'Neck simulator', 'play.sub': 'MuJoCo in your browser · nothing here talks to a real robot',
    'play.demos': 'Demos', 'play.joints': 'Joints (degrees)', 'play.pose.ready': 'Ready pose', 'play.pose.rest': 'Rest pose',
    'play.play': 'Play', 'play.pause': 'Pause', 'play.resume': 'Resume', 'play.restart': 'Restart', 'play.idle': 'Pick a demo',
    'play.toStart': 'Moving to the start', 'play.done': 'Done', 'play.dwell': '{l} (hold)',
    'play.paused': 'Paused — drag a slider or click a part to adjust the pose, then resume.',
    'play.adjusted': 'Adjusted: {j}. Resume continues from your pose.',
    'play.undo': 'Undo my changes',
    'play.cam': 'D435i view', 'play.cam.big': 'Bigger', 'play.cam.small': 'Smaller', 'play.more': 'Joints', 'play.less': 'Less',
    'play.rec': 'Record your own', 'play.rec.add': '+ Add pose', 'play.rec.n': '{n} poses', 'play.rec.play': 'Play', 'play.rec.save': 'Download JSON', 'play.rec.clear': 'Clear',
    'play.rec.name': 'My route', 'play.rec.wp': 'Pose {n}',
    'play.keys': 'Keys: ←/→ turn (J1) · ↑/↓ camera tilt (J6) · [ ] pick joint · , . move it · Space play/pause',
    'guide.1.t': 'Pick a demo', 'guide.1.b': 'Each card plays one short motion of the neck.',
    'guide.2.t': 'Pause and adjust', 'guide.2.b': 'Pause any time, drag the joint sliders (or click a part), then resume: the demo continues from your pose.',
    'guide.3.t': 'Look around', 'guide.3.b': 'Drag to orbit, scroll to zoom. The inset shows what the D435i sees.',
    'guide.next': 'Next', 'guide.done': 'Got it', 'guide.skip': 'Skip',
    'neck.prev': 'Back', 'neck.next': 'Next', 'neck.auto': 'Play all', 'neck.stop': 'Stop',
    'neck.home': 'Back to the robot', 'neck.try': 'Try it in the simulator',
    'neck.s1.t': 'Start with a standard SO101 arm',
    'neck.s1.b': 'The open-source SO101 arm from TheRobotStudio: 3D-printed links, a gripper and six STS3215 servos.',
    'neck.s2.t': 'Take it apart',
    'neck.s2.b': 'Every printed part and every servo of the arm, pulled apart along the chain.',
    'neck.s3.t': 'Only two parts change',
    'neck.s3.b': 'The lower arm and the fixed jaw are replaced. The moving jaw is not needed any more.',
    'neck.s4.t': 'Swap in the new prints',
    'neck.s4.b': 'A short J3–J4 link replaces the lower arm, and a camera-tilt housing replaces the fixed jaw. All servos are reused.',
    'neck.s5.t': 'J4 now turns sideways',
    'neck.s5.b': 'On the arm, J3 and J4 are parallel. The new link turns J4 by 90°, so the head can tilt sideways.',
    'neck.s6.t': 'Add the camera',
    'neck.s6.b': 'XLeRobot’s camera holder and a RealSense D435i go on the J6 servo, and the neck stands on XLeRobot’s tower.',
    'neck.s7.t': 'LeRobotNeck',
    'neck.s7.b': 'Six degrees of freedom: turn, lift, bend, tilt sideways, roll and look up or down.',
    'part.Rotation_Pitch': 'Shoulder bracket', 'part.Upper_Arm': 'Upper arm', 'part.Lower_Arm': 'Lower arm',
    'part.Wrist_Pitch_Roll': 'Wrist', 'part.Fixed_Jaw': 'Fixed jaw', 'part.Moving_Jaw': 'Moving jaw',
    'part.servo': 'STS3215 servo',
    'part.wallerobot_new_wrist_dof_0_1': 'New J3–J4 link', 'part.cam_tilt_housing_softfin': 'New camera-tilt housing',
    'part.tophead5': 'Camera holder (XLeRobot)', 'part.tophead6': 'RealSense D435i', 'part.base_tower': 'Tower (XLeRobot)',
    'tag.new': 'new print', 'tag.so101': 'SO101', 'tag.xlerobot': 'XLeRobot part', 'tag.out': 'removed',
    'j.neck_j1': 'J1 turn', 'j.neck_j2': 'J2 lift', 'j.neck_j3': 'J3 bend', 'j.neck_j4': 'J4 side tilt',
    'j.neck_j5': 'J5 roll', 'j.neck_j6': 'J6 camera tilt',
  },
  zh: {
    'nav.home': '首页', 'nav.play': '模拟器', 'nav.neck': '它是怎么做的', 'nav.docs': '文档', 'nav.github': 'GitHub',
    'lang.other': 'EN',
    'loading': '正在加载 3D 模型…', 'loading.pct': '正在加载 3D 模型… {p}%', 'loading.fail': '3D 模型加载失败：{e}',
    'home.kicker': '开源机器人脖子',
    'home.title': 'LeRobotNeck',
    'home.lead': '一条标准 SO101 机械臂，换上两个新打印件，就成了顶着 RealSense D435i 的 6 自由度脖子。',
    'home.stat.dof': '自由度', 'home.stat.parts': '个新打印件', 'home.stat.cam': '深度相机',
    'home.cta.play': '试试模拟器', 'home.cta.neck': '看看它是怎么做的',
    'home.hint': '拖动旋转视角 · 点脖子把它拆开',
    'home.tip': '点击拆开',
    'footer': '社区项目，基于 LeRobot、SO-ARM100 和 XLeRobot（Apache-2.0），与 Hugging Face 无隶属关系。纯仿真。',
    'play.title': '脖子模拟器', 'play.sub': 'MuJoCo 在浏览器里运行 · 不连接任何真机',
    'play.demos': '演示', 'play.joints': '关节（度）', 'play.pose.ready': '准备姿势', 'play.pose.rest': '休息位',
    'play.play': '播放', 'play.pause': '暂停', 'play.resume': '继续', 'play.restart': '重来', 'play.idle': '选一个演示',
    'play.toStart': '移动到起点', 'play.done': '完成', 'play.dwell': '{l}（停留）',
    'play.paused': '已暂停：拖动滑块或点击部件调整姿态，然后点继续。',
    'play.adjusted': '已调整：{j}。继续时会从你的姿态接着演示。',
    'play.undo': '恢复原样',
    'play.cam': 'D435i 画面', 'play.cam.big': '放大', 'play.cam.small': '缩小', 'play.more': '关节', 'play.less': '收起',
    'play.rec': '录制自己的动作', 'play.rec.add': '＋ 记录当前姿态', 'play.rec.n': '{n} 个姿态', 'play.rec.play': '播放', 'play.rec.save': '下载 JSON', 'play.rec.clear': '清空',
    'play.rec.name': '我的路线', 'play.rec.wp': '姿态 {n}',
    'play.keys': '按键：←/→ 转头（J1）· ↑/↓ 相机俯仰（J6）· [ ] 选关节 · , . 转动 · 空格 播放 / 暂停',
    'guide.1.t': '选一个演示', 'guide.1.b': '每张卡片播放脖子的一段短动作。',
    'guide.2.t': '暂停并调整', 'guide.2.b': '随时暂停，拖动关节滑块（或点击部件），再点继续：演示会从你的姿态接着进行。',
    'guide.3.t': '换个角度看', 'guide.3.b': '拖动旋转，滚轮缩放。小窗显示 D435i 看到的画面。',
    'guide.next': '下一步', 'guide.done': '知道了', 'guide.skip': '跳过',
    'neck.prev': '上一步', 'neck.next': '下一步', 'neck.auto': '自动播放', 'neck.stop': '停止',
    'neck.home': '回到机器人', 'neck.try': '在模拟器里试试',
    'neck.s1.t': '从一条标准 SO101 机械臂开始',
    'neck.s1.b': 'TheRobotStudio 的开源 SO101 机械臂：3D 打印的连杆、一个夹爪和六个 STS3215 舵机。',
    'neck.s2.t': '把它拆开',
    'neck.s2.b': '手臂的每个打印件和每个舵机，沿着关节链依次分开。',
    'neck.s3.t': '只换两个件',
    'neck.s3.b': '换掉小臂和固定夹爪；活动夹爪不再需要。',
    'neck.s4.t': '装上新打印件',
    'neck.s4.b': '一根短的 J3–J4 连杆代替小臂，一个相机俯仰外壳代替固定夹爪。所有舵机照旧使用。',
    'neck.s5.t': 'J4 改成了侧摆',
    'neck.s5.b': '机械臂上 J3 和 J4 互相平行；新连杆把 J4 转了 90°，于是头可以向两侧歪。',
    'neck.s6.t': '装上相机',
    'neck.s6.b': 'XLeRobot 的相机支架和 RealSense D435i 装在 J6 舵机上，整个脖子立在 XLeRobot 的竖塔上。',
    'neck.s7.t': 'LeRobotNeck',
    'neck.s7.b': '六个自由度：转头、抬起、弯曲、侧摆、滚转、抬头低头。',
    'part.Rotation_Pitch': '肩部支架', 'part.Upper_Arm': '大臂', 'part.Lower_Arm': '小臂',
    'part.Wrist_Pitch_Roll': '手腕', 'part.Fixed_Jaw': '固定夹爪', 'part.Moving_Jaw': '活动夹爪',
    'part.servo': 'STS3215 舵机',
    'part.wallerobot_new_wrist_dof_0_1': '新 J3–J4 连杆', 'part.cam_tilt_housing_softfin': '新相机俯仰外壳',
    'part.tophead5': '相机支架（XLeRobot）', 'part.tophead6': 'RealSense D435i', 'part.base_tower': '竖塔（XLeRobot）',
    'tag.new': '新打印件', 'tag.so101': 'SO101', 'tag.xlerobot': 'XLeRobot 零件', 'tag.out': '拿掉',
    'j.neck_j1': 'J1 转头', 'j.neck_j2': 'J2 抬起', 'j.neck_j3': 'J3 弯曲', 'j.neck_j4': 'J4 侧摆',
    'j.neck_j5': 'J5 滚转', 'j.neck_j6': 'J6 相机俯仰',
  },
};

const KEY = 'lerobotneck.lang';
const listeners = new Set();

function initial() {
  try { const s = localStorage.getItem(KEY); if (s === 'en' || s === 'zh') return s; } catch { /* storage blocked */ }
  return (navigator.language || 'en').toLowerCase().startsWith('zh') ? 'zh' : 'en';
}

let lang = initial();

export const getLang = () => lang;
export function setLang(l) {
  lang = l;
  try { localStorage.setItem(KEY, l); } catch { /* storage blocked */ }
  document.documentElement.lang = l === 'zh' ? 'zh-CN' : 'en';
  listeners.forEach((f) => f(l));
}
export const onLang = (f) => { listeners.add(f); return () => listeners.delete(f); };

export function t(key, vars = {}) {
  const s = DICT[lang][key] ?? DICT.en[key] ?? key;
  return s.replace(/\{(\w+)\}/g, (_, k) => (k in vars ? vars[k] : `{${k}}`));
}
// route-file text: plain string, or {en, zh}
export const tr = (v) => (v && typeof v === 'object' ? (v[lang] ?? v.en ?? Object.values(v)[0]) : (v ?? ''));
