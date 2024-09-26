const socket = new WebSocket('ws://localhost:8000');

socket.addEventListener('message', function (event) {
    const data = JSON.parse(event.data);
    drawField(data);
});

const k = 1;
function drawField(data) {
    const canvas = document.getElementById('footballField');
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.translate(width / 2, height / 2);
    ctx.scale(1, -1);
    ctx.clearRect(-width / 2, -height / 2, width, height);
    drawLine(ctx);
    drawRobots(ctx, data.robots);
}

function drawLine(ctx) {
    const x = 800 * k;
    const y = 500 * k;
    const goaly = 250 * k;
    const goalx = 50 * k;
    const penaltyx = 50 * k;
    const penaltyy = 270 * k;
    const Penaltyx = 120 * k;
    const Penaltyy = 350 * k;
    const centerCircleRadius = 70 * k;
    const penaltyPoint = 80 * k;
    // 边线中线
    ctx.strokeRect(-x / 2, -y / 2, x / 2, y);
    ctx.strokeRect(0, -y / 2, x / 2, y);
    // 球门
    ctx.strokeRect(-x / 2 - goalx, -goaly / 2, goalx, goaly);
    ctx.strokeRect(x / 2, -goaly / 2, goalx, goaly);
    // 小禁区
    ctx.strokeRect(-x / 2, -penaltyy / 2, penaltyx, penaltyy);
    ctx.strokeRect(x / 2 - penaltyx, -penaltyy / 2, penaltyx, penaltyy);
    // 大禁区
    ctx.strokeRect(-x / 2, -Penaltyy / 2, Penaltyx, Penaltyy);
    ctx.strokeRect(x / 2 - Penaltyx, -Penaltyy / 2, Penaltyx, Penaltyy);
    // 中圈
    ctx.beginPath();
    ctx.arc(0, 0, centerCircleRadius, 0, 2 * Math.PI);
    ctx.stroke();
    // 罚球点
    ctx.beginPath();
    ctx.arc(-x / 2 + penaltyPoint, 0, 5, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(x / 2 - penaltyPoint, 0, 5, 0, 2 * Math.PI);
    ctx.stroke();
}

function drawBall(ctx, ballx, bally) {
    const x = bally * k;
    const y = -ballx * k;
    const radius = 10 * k;

    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
    ctx.fillStyle = 'blue';
    ctx.fill();
}

function drawRobots(ctx, robots) {
    const currentTime = Date.now() / 1000;
    var ballx = 0;
    var bally = 0;
    var num = 0;
    robots.forEach((robot, i) => {
        if (robot.timestamp !== 0) {
            const x = robot.y * k;
            const y = -robot.x * k;
            const angle = (90 - robot.orientation) * Math.PI / 180;
            const arrowLength = 50 * k;
            const arrowWidth = 10 * k;

            const timeDiff = currentTime - robot.timestamp;
            const fillColor = timeDiff > 3 ? 'red' : 'green';

            ctx.save();
            ctx.translate(x, y);
            ctx.rotate(angle);
            ctx.beginPath();
            ctx.moveTo(0, -arrowLength / 2);
            ctx.lineTo(arrowWidth / 2, 0);
            ctx.lineTo(-arrowWidth / 2, 0);
            ctx.closePath();
            ctx.fillStyle = fillColor;
            ctx.fill();
            ctx.restore();

            ctx.save();
            ctx.scale(1, -1);
            ctx.font = "40px times new roman";
            const info = `${robot.id} ${robot.info}`;
            ctx.fillText(info, -800, -400 + i * 70);
            ctx.beginPath();
            ctx.moveTo(x, -y);
            ctx.lineTo(-800 + ctx.measureText(info).width, -400 + i * 70);
            ctx.stroke();
            ctx.restore();

            if (fillColor === 'green') {
                ballx += robot.x;
                bally += robot.y;
                num++;
            }
        }
    });
    if (num > 0) {
        ballx /= num;
        bally /= num;
        drawBall(ctx, ballx, bally);
    }
}

function resizeCanvas() {
    const canvas = document.getElementById('footballField');
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
}

window.addEventListener('resize', resizeCanvas);
window.addEventListener('load', resizeCanvas);