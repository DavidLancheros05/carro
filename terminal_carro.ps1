# Terminal Bluetooth para el seguidor de línea.
#   - Busca sola el puerto COM de "SeguidorV88".
#   - Si el carro se apaga, espera y se RECONECTA SOLA al encenderlo.
#   - Guarda todo en logs\AAAAMMDD_HHMMSS.log
#   - Escribe un comando y Enter (ej: ?, ver, modo 2, go, x).
#   - "salir" (o cerrar la ventana) para terminar. Ctrl+C NO la cierra.
#   - Copiar texto: seleccionarlo con el mouse y clic derecho (o Enter).
param(
  [string]$Puerto = "",               # vacío = buscar por nombre
  [string]$Nombre = "SeguidorV88"
)

$ErrorActionPreference = "Stop"

# Solo una terminal a la vez: dos compiten por el puerto y ninguna conecta
$mutex = New-Object System.Threading.Mutex($false, "Global\terminal_carro")
if (-not $mutex.WaitOne(0)) {
  Write-Host "Ya hay otra terminal del carro abierta. Usa esa o ciérrala primero." -ForegroundColor Red
  exit 1
}
$carpetaLogs = Join-Path $PSScriptRoot "logs"
New-Item -ItemType Directory -Force $carpetaLogs | Out-Null
$archivoLog = Join-Path $carpetaLogs ((Get-Date -Format "yyyyMMdd_HHmmss") + ".log")

function Buscar-Puerto {
  # El puerto SALIENTE lleva la dirección del dispositivo en su ID; se
  # compara con la del dispositivo emparejado que se llama $Nombre.
  $dev = Get-PnpDevice -ErrorAction SilentlyContinue |
         Where-Object { $_.FriendlyName -eq $Nombre -and $_.InstanceId -like 'BTHENUM\DEV_*' } |
         Select-Object -First 1
  if (-not $dev) { return $null }
  $mac = ($dev.InstanceId -split '_')[1].Substring(0, 12)
  $com = Get-CimInstance Win32_PnPEntity |
         Where-Object { $_.PNPDeviceID -like "BTHENUM*$mac*" -and $_.Name -match '\((COM\d+)\)' } |
         Select-Object -First 1
  if ($com -and $com.Name -match '\((COM\d+)\)') { return $matches[1] }
  return $null
}

function Escribir($texto, $color = $null) {
  if ($color) { Write-Host $texto -NoNewline -ForegroundColor $color }
  else        { Write-Host $texto -NoNewline }
  Add-Content -Path $archivoLog -Value $texto -NoNewline -Encoding UTF8
}

if (-not $Puerto) { $Puerto = Buscar-Puerto }
if (-not $Puerto) {
  Write-Host "No encuentro el puerto de '$Nombre'. ¿Está emparejado? Usa: -Puerto COM8" -ForegroundColor Red
  exit 1
}

Write-Host "Terminal del carro - puerto $Puerto - log en $archivoLog" -ForegroundColor Cyan
Write-Host "Escribe un comando y Enter (? = ayuda). 'salir' para terminar." -ForegroundColor Cyan
Write-Host "Para copiar: selecciona con el mouse y clic derecho.`n" -ForegroundColor Cyan

$linea = ""
$avisadoEspera = ""
[Console]::TreatControlCAsInput = $true   # Ctrl+C no cierra la terminal

while ($true) {
  # ---------- Conectar (reintenta hasta que el carro esté encendido) ----------
  $sp = New-Object System.IO.Ports.SerialPort $Puerto, 115200
  $sp.ReadTimeout  = 200
  $sp.WriteTimeout = 1500
  $sp.NewLine = "`n"
  try {
    $sp.Open()
  } catch {
    # Mostrar el motivo cada vez que cambia (ayuda a saber qué pasa)
    $motivo = $_.Exception.InnerException.Message
    if (-not $motivo) { $motivo = $_.Exception.Message }
    if ($avisadoEspera -ne $motivo) {
      Escribir "`r`n[esperando al carro en $Puerto... ($motivo)]`r`n" "Yellow"
      if ($motivo -match 'denegado|denied') {
        Escribir "[Otro programa tiene abierto ${Puerto}: cierra el Monitor Serie del Arduino IDE, PuTTY u otra terminal]`r`n" "Yellow"
      }
      $avisadoEspera = $motivo
    }
    $sp.Dispose()
    Start-Sleep -Milliseconds 1500
    continue
  }
  $avisadoEspera = ""
  Escribir "`r`n[CONECTADO $(Get-Date -Format 'HH:mm:ss')]`r`n" "Green"

  # ---------- Recibir / enviar hasta que se corte ----------
  $ultimoLatido = Get-Date
  try {
    while ($true) {
      $datos = $sp.ReadExisting()
      if ($datos) {
        if ($linea) { Write-Host "" }                 # no pisar lo que estás escribiendo
        Escribir $datos
        if ($linea) { Write-Host "> $linea" -NoNewline -ForegroundColor Cyan }
      }

      while ([Console]::KeyAvailable) {
        $k = [Console]::ReadKey($true)
        if ($k.Key -eq 'Enter') {
          Write-Host ""
          if ($linea.Trim() -eq 'salir') {
            Escribir "`r`n[TERMINAL CERRADA $(Get-Date -Format 'HH:mm:ss')]`r`n" "Cyan"
            $sp.Close()
            exit 0
          }
          $sp.WriteLine($linea)
          Add-Content -Path $archivoLog -Value "`r`n> $linea`r`n" -NoNewline -Encoding UTF8
          $linea = ""
          $ultimoLatido = Get-Date
        } elseif ($k.Key -eq 'Backspace') {
          if ($linea.Length -gt 0) { $linea = $linea.Substring(0, $linea.Length - 1); Write-Host "`b `b" -NoNewline }
        } elseif ([int]$k.KeyChar -ge 32) {       # ignora Ctrl+C y otras teclas de control
          if (-not $linea) { Write-Host "> " -NoNewline -ForegroundColor Cyan }
          $linea += $k.KeyChar
          Write-Host $k.KeyChar -NoNewline -ForegroundColor Cyan
        }
      }

      # Latido: una línea vacía (el carro la ignora). Si no se puede enviar,
      # el carro se apagó -> reconectar.
      if (((Get-Date) - $ultimoLatido).TotalSeconds -ge 2) {
        $sp.WriteLine("")
        $ultimoLatido = Get-Date
      }
      Start-Sleep -Milliseconds 20
    }
  } catch {
    Escribir "`r`n[SE CORTO LA CONEXION $(Get-Date -Format 'HH:mm:ss') - reconectando...]`r`n" "Yellow"
  } finally {
    try { $sp.Close() } catch {}
    $sp.Dispose()
  }
  Start-Sleep -Milliseconds 1000
}
