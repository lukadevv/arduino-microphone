import asyncio
import pyaudio
import numpy as np
import websockets

# Audio config (debe coincidir con ESP32)
SAMPLE_RATE = 8000
SAMPLE_FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK = 512

# Ganancia digital (ajustá este valor según tu necesidad)
GAIN = 17.0  # Podés probar con 2.0, 2.5, 3.0, etc.

# Inicializar PyAudio para salida
p = pyaudio.PyAudio()
stream_out = p.open(format=SAMPLE_FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    output=True,
                    frames_per_buffer=CHUNK)

async def audio_handler(websocket, path=None):
    print(f"Cliente conectado desde {websocket.remote_address}. Ruta: {path}")
    try:
        async for message in websocket:
            if not message:
                continue  # Mensaje vacío, lo ignoramos

            try:
                # Convertir bytes a array numpy
                audio_data = np.frombuffer(message, dtype=np.int16)

                # Aplicar ganancia con clipping
                processed_audio = (audio_data * GAIN).clip(-32768, 32767).astype(np.int16)

                # Medir volumen (opcional, útil para debug)
                rms = np.sqrt(np.mean(audio_data**2))
                print(f"RMS: {int(rms)}")

                # Reproducir
                try:
                    stream_out.write(processed_audio.tobytes())
                except Exception as e:
                    print(f"[ERROR] PyAudio playback: {e}")

                # Reenviar (opcional)
                await websocket.send(processed_audio.tobytes())

            except Exception as e:
                print(f"[ERROR] Procesando audio: {e}")

    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Conexión cerrada inesperadamente: {e}")
    except Exception as e:
        print(f"[ERROR] General: {e}")
    finally:
        print("Cliente desconectado")

async def main():
    async with websockets.serve(audio_handler, "0.0.0.0", 8765):
        print("Servidor WebSocket escuchando en puerto 8765...")
        await asyncio.Event().wait()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServidor detenido manualmente.")
    finally:
        stream_out.stop_stream()
        stream_out.close()
        p.terminate()
