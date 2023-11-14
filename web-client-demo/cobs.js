window.cobs = {}

window.cobs.decode = function(input) {
  let output = []

  // the first byte to decode denotes where the next zero is
  // 
  // when the next zero byte is further than 255 bytes away,
  // cobs inserts a distance of 255 after which another overhead
  // byte will denote the true distance minus that 255
  cobs_zero_distance = input[0];
  last_cobs_i = 0;

  next_cobs_i = input[0];
  last_cobs_i = 0;

  for (let i = 1; i < input.length; i++) {
    if (i != next_cobs_i) {
      output.push(input[i])
    } else {
      if (next_cobs_i - last_cobs_i == 0xFF) {
        // When the next zero byte is further than 254 bytes away,
        // cobs write value 255 (the maximum), denoting to travel
        // that distance for the next cobs distance byte.
        // In this case, we should not write a 0x00 in place of the
        // 0xFF byte.
      } else if (input[i] == 0x00) {
        // the end
      } else {
        output.push(0x00)
      }

      last_cobs_i = i
      next_cobs_i = i + input[i]
    }
  }

  return new Uint8Array(output);
}