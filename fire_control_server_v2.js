const express = require('express');
const http = require('http');
const { Server } = require("socket.io");
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { spawn } = require('child_process');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

const PORT = 3000;

// --- 중요! --- STM32 보드가 연결된 실제 COM 포트 번호로 변경하세요。
const SERIAL_PORT_PATH = 'COM6'; // 사용자 설정에 따라 COM6 유지

let pythonStreamerProcess = null;
let fireAlertSentToClient = false; // 웹 클라이언트에 fire_alert가 전송되었는지 추적

// 루트 경로 요청 시 fire_control_v2.html을 전송
app.get('/', (req, res) => {
  res.sendFile(__dirname + '/public/fire_control_v2.html');
});

// 정적 파일(HTML, CSS, 클라이언트 JS)을 제공하기 위한 폴더 설정
app.use(express.static('public'));

// 시리얼 포트 설정
const serialPort = new SerialPort({
  path: SERIAL_PORT_PATH,
  baudRate: 115200,
  autoOpen: false, // 에러 처리를 위해 수동으로 엽니다.
});

// 시리얼 포트 열기 시도
serialPort.open((err) => {
  if (err) {
    console.log(`경고: ${SERIAL_PORT_PATH}에 연결할 수 없습니다. ${err.message}`);
    console.log('RC카와의 통신 없이 서버를 시작합니다. 웹 UI는 동작하지만, RC카 제어 및 화재 신호 수신은 불가능합니다.');
  } else {
    console.log(`${SERIAL_PORT_PATH}에 성공적으로 연결되었습니다.`);
  }
});

const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

// STM32로부터 데이터 수신 처리
parser.on('data', (data) => {
  const signal = data.toString().trim();
  console.log(`STM32로부터 수신된 신호: ${signal}`);

  // 'X' 신호 수신 시 화재 경보 로직 실행
  if (signal === 'X') {
    
    // 파이썬 스트리머가 이미 실행 중이 아니라면 실행합니다.
    if (!pythonStreamerProcess) {
      console.log('Python 카메라 스트리머를 시작합니다...');
      pythonStreamerProcess = spawn('python', ['fire_follower_streamer.py']);

      // 파이썬 스크립트 실행 중 오류 발생 시 처리
      pythonStreamerProcess.on('error', (err) => {
        console.error(`[Python Streamer LAUNCH ERROR]: ${err}`);
      });

      // 파이썬 스크립트의 표준 출력을 감지합니다。

      // 파이썬 스크립트가 켜질 시간을 벌기 위해 10초 지연 후 화면 전환
      console.log('화재 감지! 원격 제어 프로그램을 준비합니다...');
      setTimeout(() => {
        console.log('웹 클라이언트에 화면 전환을 지시합니다.');
        io.emit('fire_alert');
        fireAlertSentToClient = true; // 플래그 설정
      }, 10000); // 10초 딜레이

      // --- 파이썬 프로세스 관련 로그 출력 ---
      pythonStreamerProcess.stdout.on('data', (stdoutData) => {
        console.log(`[Python Streamer]: ${stdoutData.toString().trim()}`);
      });
      pythonStreamerProcess.stderr.on('data', (stderrData) => {
        console.error(`[Python Streamer ERROR]: ${stderrData}`);
      });
      pythonStreamerProcess.on('close', (code) => {
        console.log(`Python 스트리머 프로세스가 종료되었습니다. 종료 코드: ${code}`);
        pythonStreamerProcess = null;
        fireAlertSentToClient = false; // 프로세스 종료 시 플래그 초기화
      });
    }
  }
});

// 웹 클라이언트와의 웹소켓 연결 처리
io.on('connection', (socket) => {
  console.log('웹 클라이언트가 연결되었습니다.');

  // 새로 연결된 클라이언트에게 현재 상태를 알림
  if (pythonStreamerProcess && fireAlertSentToClient) {
    console.log('새 클라이언트에게 화재 경보 상태를 즉시 알립니다.');
    socket.emit('fire_alert');
  }

  // 클라이언트로부터 'move_start' 이벤트 수신 (RC카 조종 시작)
  socket.on('move_start', (command) => {
    console.log(`웹으로부터 받은 조종 명령 시작: ${command}`);
    if (serialPort.isOpen) {
      serialPort.write(command, (err) => {
        if (err) {
          return console.log('시리얼 포트 쓰기 오류: ', err.message);
        }
      });
    } else {
      console.log('경고: RC카가 연결되지 않아 조종 명령을 보낼 수 없습니다.');
    }
  });

  // 클라이언트로부터 'move_stop' 이벤트 수신 (RC카 조종 중지)
  socket.on('move_stop', (command) => {
    console.log(`웹으로부터 받은 조종 명령 중지: ${command}`);
    if (serialPort.isOpen) {
      serialPort.write(command, (err) => {
        if (err) {
          return console.log('시리얼 포트 쓰기 오류: ', err.message);
        }
      });
    } else {
      console.log('경고: RC카가 연결되지 않아 조종 명령을 보낼 수 없습니다.');
    }
  });

  // 클라이언트로부터 'extinguish_fire' 이벤트 수신
  socket.on('extinguish_fire', () => {
    console.log('웹으로부터 소화 명령 수신.');
    if (serialPort.isOpen) {
      serialPort.write('E', (err) => { // 'E'는 소화 명령을 나타내는 새로운 시리얼 명령어
        if (err) {
          return console.log('시리얼 포트 쓰기 오류: ', err.message);
        }
        console.log('소화 명령 (E)을 STM32로 전송했습니다.');
      });
    } else {
      console.log('경고: RC카가 연결되지 않아 소화 명령을 보낼 수 없습니다.');
    }
  });

  // --- NEW: 클라이언트로부터 'patrol' 이벤트 수신 ---
  socket.on('patrol', () => {
    console.log('웹으로부터 순찰 명령 수신.');
    if (serialPort.isOpen) {
      serialPort.write('p', (err) => {
        if (err) {
          return console.log('시리얼 포트 쓰기 오류: ', err.message);
        }
        console.log('순찰 명령 (p)을 STM32로 전송했습니다.');
      });
    } else {
      console.log('경고: RC카가 연결되지 않아 순찰 명령을 보낼 수 없습니다.');
    }
  });

  // --- NEW: 클라이언트로부터 'clear_fire_alert' 이벤트 수신 ---
  socket.on('clear_fire_alert', () => {
    console.log('웹으로부터 화재 경보 초기화 명령 수신.');
    if (pythonStreamerProcess) {
      console.log('파이썬 스트리머 프로세스를 종료합니다.');
      pythonStreamerProcess.kill(); // 파이썬 프로세스 종료
      pythonStreamerProcess = null;
    }
    fireAlertSentToClient = false; // 플래그 초기화
  });

  socket.on('disconnect', () => {
    console.log('웹 클라이언트 연결이 끊어졌습니다.');
    // 모든 클라이언트가 연결을 끊으면 파이썬 스트리머도 종료
    if (io.engine.clientsCount === 0 && pythonStreamerProcess) {
      console.log('모든 웹 클라이언트 연결이 끊어져 파이썬 스트리머를 종료합니다.');
      pythonStreamerProcess.kill();
      pythonStreamerProcess = null;
      fireAlertSentToClient = false; // 프로세스 종료 시 플래그 초기화
    }
  });
});

// 서버 시작
server.listen(PORT, () => {
  console.log(`서버가 http://localhost:${PORT} 에서 실행 중입니다.`);
});