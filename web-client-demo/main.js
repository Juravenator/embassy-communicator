let ApiMessage;
protobuf.load("proto/api.proto", (err, root) => {
  if (err) throw err;

  ApiMessage = root.lookupType("api.Message");
  let EmptyMessage = root.lookupType("api.v1.Request.Empty");
  let payload = {
    v1_request: {configureDsp: {sample_rate: 1, block_size: 2}}
  }
  err = ApiMessage.verify(payload);
  if (err) {
    throw Error(err);
  }
  message = ApiMessage.create(payload);
  b = ApiMessage.encode(message).finish();
  console.log({ApiMessage, message, err, b})
});

let button = document.querySelector("button");

navigator.serial.addEventListener("connect", (e) => {
  // Connect to `e.target` or add it to a list of available ports.
});

navigator.serial.addEventListener("disconnect", (e) => {
  // Remove `e.target` from the list of available ports.
});

navigator.serial.getPorts().then((ports) => {
  // Initialize the list of available ports with `ports` on page load.
});

button.addEventListener("click", () => {
  const usbVendorId = 0xc0de;
  navigator.serial
    .requestPort({ filters: [{ usbVendorId }] })
    .then(port => port.open({baudRate: 115200}).then(() => port))
    .then(async (port) => {
      console.dir(port)
      button.parentElement.removeChild(button);
      const writer = port.writable.getWriter();
      const request_bytes = new Uint8Array([0x4, 0x12, 0x2, 0xa, 0x5, 0x22, 0x3f, 0x7c, 0xec, 0x0]);
      console.log("writing", request_bytes);
      writer.ready
        .then(() => writer.write(request_bytes))
        .then(() => {
          console.log("Chunk written to sink.");
        })
        .catch((err) => {
          console.error("Chunk error:", err);
        });

      while (port.readable) {
        const reader = port.readable.getReader();
        while (true) {
          try {
            const { value, done } = await reader.read();
            console.log("received", {value, done})
            let decoded = cobs.decode(value);
            console.log("decoded", decoded);

            // strip off CRC and check
            let crc_input = new Uint8Array(decoded.buffer.slice(-4));
            decoded = decoded.slice(0, -4);
            crc_input.reverse(); // endianness
            crc_input = new Uint32Array(crc_input.buffer)[0] | 0; // | 0 converts to signed
            let crc_check = CRC32.buf(decoded)
            console.log({crc_input, crc_check});
            if (crc_input != crc_check) {
              throw Error("CRC mismatch")
            } else {
              console.log("CRC match");
            }

            decoded = ApiMessage.decode(decoded);
            decoded = ApiMessage.toObject(decoded);
            let decoded_str = JSON.stringify(decoded, undefined, 2);
            console.log(decoded);

            const pre = document.createElement('pre');
            const code = document.createElement('code');

            code.innerText = decoded_str;
            pre.appendChild(code)
            document.body.appendChild(pre)

          } catch(err) {
            console.error("read error", err);
            await reader.releaseLock();
            break;
          }
        }
        break;
      }
    })
    .catch((e) => {
      // The user didn't select a port.
      console.error("no port selected", e);
    });
});