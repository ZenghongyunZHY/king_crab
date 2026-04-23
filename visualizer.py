#plotly 画图

from dash import Dash, dcc, html, Input, Output
import kinematics
import math
import plotly.graph_objects as go


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qw, qx, qy, qz]

Run_posture_control = kinematics.run_posture_control()  # 创建姿态控制实例
Run_posture_control.post_init()  # 初始化姿态控制器
app = Dash(__name__)
app.layout = html.Div([
    html.H2("姿态控制可视化"),
    html.H4("当前姿态 IMU"),
    html.Div("Roll:"),
    dcc.Slider(id='IMU_roll-slider', min=-30, max=30, step=1, value=0),
    html.Div("Pitch:"),
    dcc.Slider(id='IMU_pitch-slider', min=-30, max=30, step=1, value=0),
    html.Div("Yaw:"),   
    dcc.Slider(id='IMU_yaw-slider', min=-30, max=30, step=1, value=0),

    html.H4("目标姿态"),
    html.Div("Roll:"),
    dcc.Slider(id='target-roll-slider', min=-30, max=30, step=1, value=0),
    html.Div("Pitch:"),
    dcc.Slider(id='target-pitch-slider', min=-30, max=30, step=1, value=0),
    html.Div("Yaw:"),
    dcc.Slider(id='target-yaw-slider', min=-30, max=30, step=1, value=0),

    html.H4("输出"),
    dcc.Graph(id='robot-graph'),
    html.Div(id='output-div')
])



@app.callback(
    Output('output-div', 'children'),
    Output('robot-graph', 'figure'),
    Output('IMU_roll-slider', 'value'),
    Output('IMU_pitch-slider', 'value'),
    Output('IMU_yaw-slider', 'value'),

    Input('IMU_roll-slider', 'value'),
    Input('IMU_pitch-slider', 'value'),
    Input('IMU_yaw-slider', 'value'),
    Input('target-roll-slider', 'value'),
    Input('target-pitch-slider', 'value'),
    Input('target-yaw-slider', 'value')
)

def update_output(IMU_roll, IMU_pitch, IMU_yaw, target_roll, target_pitch, target_yaw):
    current_quaternion = euler_to_quaternion(IMU_roll, IMU_pitch, IMU_yaw)
    target_quaternion = euler_to_quaternion(target_roll, target_pitch, target_yaw)

    hexapod,angle_arr= Run_posture_control.run(current_quaternion, target_quaternion,[0,0,0])

    lines = []
    for i, ang in enumerate(angle_arr, start=1):
        deg = [round(math.degrees(a), 2) for a in ang]
        lines.append(f"Leg {i} angles: {deg}")

    fig = build_figure(hexapod)

    #Run_posture_control.post_init()  # 重置姿态控制器状态

    #假设它能完美达到目标姿态，重置当前姿态为目标姿态

    IMU_roll = target_roll
    IMU_pitch = target_pitch
    IMU_yaw = target_yaw

    Run_posture_control.q_prev = target_quaternion

    return html.Pre("\n".join(lines)), fig, IMU_roll, IMU_pitch, IMU_yaw

def build_figure(hexapod):
    fig = go.Figure()

    body_points = hexapod.body_endpoint
    joint_points = hexapod.per_leg_point

    def split_xyz(points):
        return (
            [p[0] for p in points],
            [p[1] for p in points],
            [p[2] for p in points],
        )

    # 机身端点顺序：左前 -> 左中 -> 左后 -> 右后 -> 右中 -> 右前 -> 左前
    body_order = [0, 2, 4, 5, 3, 1, 0]
    body_outline = [body_points[i] for i in body_order]
    x, y, z = split_xyz(body_outline)
    fig.add_trace(go.Scatter3d(
        x=x,
        y=y,
        z=z,  
        mode="lines+markers",
        name="Body outline",
        line=dict(color="#2563eb", width=7),
        marker=dict(size=5, color="#1d4ed8"),
        hovertemplate="Body endpoint<br>x=%{x:.2f}<br>y=%{y:.2f}<br>z=%{z:.2f}<extra></extra>",
    ))

    body_surface = [body_points[i] for i in body_order[:-1]]
    x, y, z = split_xyz(body_surface)
    fig.add_trace(go.Mesh3d(
        x=x,
        y=y,
        z=z,
        i=[0, 0, 0, 0],
        j=[1, 2, 3, 4],
        k=[2, 3, 4, 5],
        name="Body surface",
        color="#60a5fa",
        opacity=0.2,
        hoverinfo="skip",
        showlegend=False,
    ))

    leg_colors = ["#dc2626", "#f97316", "#16a34a", "#0891b2", "#7c3aed", "#db2777"]
    for leg_index in range(6):
        leg_start = body_points[leg_index]
        leg_joints = joint_points[3 * leg_index:3 * leg_index + 3]
        leg_trace_points = [leg_start] + leg_joints
        x, y, z = split_xyz(leg_trace_points)
        fig.add_trace(go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines+markers",
            name=f"Leg {leg_index + 1}",
            line=dict(color=leg_colors[leg_index], width=6),
            marker=dict(size=[5, 4, 4, 6], color=leg_colors[leg_index]),
            hovertemplate=(
                f"Leg {leg_index + 1}<br>"
                "x=%{x:.2f}<br>y=%{y:.2f}<br>z=%{z:.2f}<extra></extra>"
            ),
        ))

    foot_points = [joint_points[3 * i + 2] for i in range(6)]
    x, y, z = split_xyz(foot_points)
    fig.add_trace(go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode="markers",
        name="Feet",
        marker=dict(size=7, color="#111827", symbol="circle"),
        hovertemplate="Foot<br>x=%{x:.2f}<br>y=%{y:.2f}<br>z=%{z:.2f}<extra></extra>",
    ))

    all_points = body_points + joint_points
    axis_len = max(
        max(abs(p[0]), abs(p[1]), abs(p[2])) for p in all_points
    ) * 0.35
    axes = [
        ("X", [0, axis_len], [0, 0], [0, 0], "#ef4444"),
        ("Y", [0, 0], [0, axis_len], [0, 0], "#22c55e"),
        ("Z", [0, 0], [0, 0], [0, axis_len], "#3b82f6"),
    ]
    for name, x, y, z, color in axes:
        fig.add_trace(go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines+text",
            name=f"{name} axis",
            line=dict(color=color, width=4),
            text=["", name],
            textposition="top center",
            hoverinfo="skip",
            showlegend=False,
        ))

    xs, ys, zs = split_xyz(all_points)
    padding = 5
    fig.update_layout(
        title="Hexapod posture",
        height=720,
        margin=dict(l=0, r=0, t=45, b=0),
        scene=dict(
            xaxis=dict(title="X", range=[min(xs) - padding, max(xs) + padding]),
            yaxis=dict(title="Y", range=[min(ys) - padding, max(ys) + padding]),
            zaxis=dict(title="Z", range=[min(zs) - padding, max(zs) + padding]),
            aspectmode="data",
            camera=dict(eye=dict(x=1.55, y=1.55, z=1.1)),
        ),
        legend=dict(
            x=0.01,
            y=0.99,
            bgcolor="rgba(255,255,255,0.65)",
        ),
    )

    return fig
if __name__ == "__main__":
    app.run(debug = True)
