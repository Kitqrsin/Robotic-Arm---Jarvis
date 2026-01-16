// --- Configuration ---
const API_BASE = window.location.origin;

// --- State ---
let armState = {
    s1: 90, // Base
    s2: 94,  // Shoulder
    s3: 94,  // Elbow
    s4: 72,  // Forearm
    s5: 60,  // Wrist
    s6: 45   // Gripper
};

let isConnected = false;
let oeEnabled = false;  // Track OE pin state
let servoSpeeds = { s1: 100, s2: 100, s3: 100, s4: 100, s5: 100, s6: 100 };  // Individual servo speeds (ms delay per degree)
let isMoving = { s1: false, s2: false, s3: false, s4: false, s5: false, s6: false };  // Per-motor movement locks
let chainQueue = [];  // Poses in chain (max 5)
let isChainRunning = false;  // Chain execution state

// Saved poses now loaded from server
let savedPoses = [];

// --- API Functions ---

async function sendServoCommand(servoId, angle) {
    try {
        const response = await fetch(`${API_BASE}/api/servo/${servoId}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ angle: angle })
        });
        
        const data = await response.json();
        if (!data.success) {
            console.error('Servo command failed:', data.error);
        }
        return data.success;
    } catch (error) {
        console.error('Network error:', error);
        updateConnectionStatus(false);
        return false;
    }
}

async function checkStatus() {
    try {
        const response = await fetch(`${API_BASE}/api/status`);
        const data = await response.json();
        updateConnectionStatus(data.online);
        updateOEStatus(data.oe_enabled);
        return data;
    } catch (error) {
        updateConnectionStatus(false);
        return null;
    }
}

async function loadLastPosition() {
    try {
        const response = await fetch(`${API_BASE}/api/last_position`);
        const data = await response.json();
        
        if (data.success && data.position) {
            const position = data.position;
            
            // Update armState
            armState.s1 = position.s1 || 90;
            armState.s2 = position.s2 || 94;
            armState.s3 = position.s3 || 94;
            armState.s4 = position.s4 || 72;
            armState.s5 = position.s5 || 60;
            armState.s6 = position.s6 || 45;
            
            // Update UI sliders and displays
            for (let key in armState) {
                const slider = document.getElementById(key);
                const servoId = parseInt(key.substring(1));
                if (slider) {
                    slider.value = armState[key];
                    document.getElementById(`val-s${servoId}`).innerText = armState[key] + '°';
                }
            }
            
            logData('Loaded last known position');
            return true;
        }
        return false;
    } catch (error) {
        console.error('Failed to load last position:', error);
        return false;
    }
}

async function toggleOE() {
    // Check if any servo is currently moving
    const anyMoving = Object.values(isMoving).some(moving => moving);
    if (anyMoving) {
        logData('⚠️ Cannot toggle motors during movement', true);
        return;
    }
    
    try {
        const newState = !oeEnabled;
        const response = await fetch(`${API_BASE}/api/oe_toggle`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ enable: newState })
        });
        
        const data = await response.json();
        if (data.success) {
            updateOEStatus(data.oe_enabled);
            logData(data.message);
            
            // Safety: Reset sliders to initial positions when motors are enabled
            if (data.oe_enabled) {
                const initialPositions = {
                    s1: 90, // Base
                    s2: 94,  // Shoulder
                    s3: 94,  // Elbow
                    s4: 72,  // Forearm
                    s5: 60,  // Wrist
                    s6: 45   // Gripper
                };
                
                // Lock all servos
                const servosToMove = [];
                for (let key in initialPositions) {
                    const servoId = parseInt(key.substring(1));
                    servosToMove.push(servoId);
                    setMovementLock(servoId, true);
                    
                    const slider = document.getElementById(key);
                    if (slider) {
                        slider.value = initialPositions[key];
                        document.getElementById(`val-s${servoId}`).innerText = initialPositions[key] + '°';
                    }
                }
                
                // Move motors gradually to safe positions
                logData('Moving to safe positions...');
                const moves = [];
                for (let key in initialPositions) {
                    const servoId = parseInt(key.substring(1));
                    moves.push(moveServoGradually(servoId, initialPositions[key]));
                }
                
                try {
                    await Promise.all(moves);
                    logData('Safe position reached');
                } finally {
                    // Unlock all servos
                    servosToMove.forEach(servoId => setMovementLock(servoId, false));
                }
            }
        } else {
            alert('Failed to toggle motors: ' + data.error);
        }
    } catch (error) {
        console.error('OE toggle failed:', error);
        alert('Network error: Could not toggle motors');
    }
}

function updateOEStatus(enabled) {
    oeEnabled = enabled;
    const statusEl = document.getElementById('oe-status');
    const btnEl = document.getElementById('oe-toggle-btn');
    
    if (enabled) {
        statusEl.textContent = 'ENABLED';
        statusEl.className = 'text-emerald-400 font-bold';
        btnEl.textContent = 'DISABLE MOTORS';
        btnEl.className = 'bg-orange-600 hover:bg-orange-700 text-white px-4 py-2 rounded shadow-lg font-bold transition-colors';
        // Unlock all sliders when motors are enabled
        unlockAllSliders();
        unlockPoseButtons();
    } else {
        statusEl.textContent = 'DISABLED';
        statusEl.className = 'text-red-400 font-bold';
        btnEl.textContent = 'ENABLE MOTORS';
        btnEl.className = 'bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded shadow-lg font-bold transition-colors';
        // Lock all sliders when motors are disabled
        lockAllSliders();
        lockPoseButtons();
    }
}

function lockAllSliders() {
    const sliders = document.querySelectorAll('input[type="range"]');
    sliders.forEach(slider => {
        if (slider.id.startsWith('s')) {  // Only lock servo sliders, not delay slider
            slider.disabled = true;
            slider.style.opacity = '0.3';
        }
    });
}

function unlockAllSliders() {
    const sliders = document.querySelectorAll('input[type="range"]');
    sliders.forEach(slider => {
        if (slider.id.startsWith('s')) {  // Only unlock servo sliders
            const servoKey = slider.id;
            // Only unlock if not currently moving
            if (!isMoving[servoKey]) {
                slider.disabled = false;
                slider.style.opacity = '1';
            }
        }
    });
}

function lockPoseButtons() {
    // Lock save button
    const saveBtn = document.querySelector('button[onclick*="saveCurrentPose"]');
    if (saveBtn) {
        saveBtn.disabled = true;
        saveBtn.style.opacity = '0.3';
    }
    
    // Lock all load pose buttons
    const loadBtns = document.querySelectorAll('button[onclick*="applyState"]');
    loadBtns.forEach(btn => {
        btn.disabled = true;
        btn.style.opacity = '0.3';
    });
    
    // Lock delete buttons
    const deleteBtns = document.querySelectorAll('button[onclick*="deletePose"]');
    deleteBtns.forEach(btn => {
        btn.disabled = true;
        btn.style.opacity = '0.3';
    });
    
    // Update chain buttons
    updateChainButtons();
}

function unlockPoseButtons() {
    // Unlock save button
    const saveBtn = document.querySelector('button[onclick*="saveCurrentPose"]');
    if (saveBtn) {
        saveBtn.disabled = false;
        saveBtn.style.opacity = '1';
    }
    
    // Unlock all load pose buttons
    const loadBtns = document.querySelectorAll('button[onclick*="applyState"]');
    loadBtns.forEach(btn => {
        btn.disabled = false;
        btn.style.opacity = '1';
    });
    
    // Unlock delete buttons
    const deleteBtns = document.querySelectorAll('button[onclick*="deletePose"]');
    deleteBtns.forEach(btn => {
        btn.disabled = false;
        btn.style.opacity = '1';
    });
    
    // Update chain buttons
    updateChainButtons();
}

function updateConnectionStatus(connected) {
    isConnected = connected;
    const indicator = document.getElementById('status-indicator');
    if (connected) {
        indicator.textContent = 'ONLINE';
        indicator.className = 'text-emerald-400 font-bold';
    } else {
        indicator.textContent = 'OFFLINE';
        indicator.className = 'text-red-400 font-bold';
    }
}

// --- UI Logic ---

function setMovementLock(servoId, locked) {
    const servoKey = `s${servoId}`;
    isMoving[servoKey] = locked;
    
    const slider = document.getElementById(servoKey);
    if (slider) {
        // Only change slider state if motors are enabled
        if (oeEnabled) {
            slider.disabled = locked;
            slider.style.opacity = locked ? '0.5' : '1';
        }
        // If motors are disabled, keep slider locked (will be set by lockAllSliders)
    }
    
    // Lock pose buttons if ANY servo is moving
    const anyMoving = Object.values(isMoving).some(moving => moving);
    const buttons = document.querySelectorAll('button');
    buttons.forEach(button => {
        if (!button.onclick || !button.onclick.toString().includes('emergencyStop')) {
            button.disabled = anyMoving;
            button.style.opacity = anyMoving ? '0.5' : '1';
        }
    });
}

function updateServoSpeed(servoId, value) {
    const servoKey = `s${servoId}`;
    servoSpeeds[servoKey] = parseInt(value);
    document.getElementById(`speed-val-s${servoId}`).innerText = value + 'ms';
}

async function moveServoGradually(servoId, targetAngle) {
    const currentAngle = armState[`s${servoId}`];
    const step = targetAngle > currentAngle ? 1 : -1;
    const servoKey = `s${servoId}`;
    const delay = servoSpeeds[servoKey] || 100;  // Use servo-specific speed
    
    // Move one degree at a time with servo-specific delay
    for (let angle = currentAngle; step > 0 ? angle <= targetAngle : angle >= targetAngle; angle += step) {
        armState[servoKey] = angle;
        await sendServoCommand(servoId, angle);
        await new Promise(resolve => setTimeout(resolve, delay));
    }
}

function updateArm(servoId, value) {
    const servoKey = `s${servoId}`;
    
    // Prevent movement if motors are disabled
    if (!oeEnabled) {
        logData(`⚠️ Motors disabled - enable motors first`, true);
        // Reset slider to current position
        const slider = document.getElementById(servoKey);
        if (slider) {
            slider.value = armState[servoKey];
            document.getElementById(`val-s${servoId}`).innerText = armState[servoKey] + '°';
        }
        return;
    }
    
    if (isMoving[servoKey]) {
        logData(`⚠️ Servo ${servoId} blocked - still moving`, true);
        // Reset slider to current position
        const slider = document.getElementById(servoKey);
        if (slider) {
            slider.value = armState[servoKey];
            document.getElementById(`val-s${servoId}`).innerText = armState[servoKey] + '°';
        }
        return;
    }
    
    value = parseInt(value);
    
    // Move gradually to target with per-servo lock
    setMovementLock(servoId, true);
    moveServoGradually(servoId, value)
        .then(() => {
            setMovementLock(servoId, false);
            logData(`Servo ${servoId} → ${value}° complete`);
        })
        .catch(err => {
            setMovementLock(servoId, false);
            console.error('Movement error:', err);
        });
}

// Apply a full state object to the arm
async function applyState(newState) {
    // Prevent pose loading if motors are disabled
    if (!oeEnabled) {
        logData('⚠️ Motors disabled - enable motors to load pose', true);
        return;
    }
    
    // Check if any servo is currently moving
    const anyMoving = Object.values(isMoving).some(moving => moving);
    if (anyMoving) {
        logData('⚠️ Pose blocked - servos still moving', true);
        return;
    }
    
    const moves = [];
    const servosToMove = [];
    
    for (let key in newState) {
        if (newState[key] !== undefined) {
            const servoId = parseInt(key.substring(1));
            const slider = document.getElementById(key);
            if (slider) {
                slider.value = newState[key];
                document.getElementById(`val-s${servoId}`).innerText = newState[key] + '°';
                servosToMove.push(servoId);
                setMovementLock(servoId, true);
                moves.push(moveServoGradually(servoId, newState[key]));
            }
        }
    }
    
    try {
        // Execute all moves in parallel
        await Promise.all(moves);
        logData('Pose loaded successfully');
    } catch (err) {
        console.error('Pose loading error:', err);
    } finally {
        // Unlock all servos that were moved
        servosToMove.forEach(servoId => setMovementLock(servoId, false));
    }
}

// --- Save/Load Logic ---

async function loadSavedPoses() {
    try {
        const response = await fetch(`${API_BASE}/api/saved_poses`);
        const data = await response.json();
        if (data.success) {
            savedPoses = data.poses;
            renderSavedPoses();
        }
    } catch (error) {
        console.error('Failed to load poses:', error);
    }
}

async function saveCurrentPose() {
    // Prevent saving if motors are disabled
    if (!oeEnabled) {
        logData('⚠️ Motors disabled - enable motors to save pose', true);
        return;
    }
    
    const nameInput = document.getElementById('pose-name-input');
    const name = nameInput.value.trim();

    if (!name) {
        alert("Please enter a name for this position.");
        return;
    }

    try {
        const response = await fetch(`${API_BASE}/api/saved_poses`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                name: name,
                state: armState
            })
        });
        
        const data = await response.json();
        if (data.success) {
            nameInput.value = '';
            logData(`Saved pose: ${name}`);
            await loadSavedPoses();  // Reload from server
        } else {
            alert('Failed to save: ' + data.error);
        }
    } catch (error) {
        console.error('Save failed:', error);
        alert('Network error: Could not save pose');
    }
}

function loadPose(index) {
    const pose = savedPoses[index];
    if (pose) {
        applyState(pose.state);
    }
}

async function deletePose(index) {
    const pose = savedPoses[index];
    if (!pose) return;
    
    if (confirm('Delete position "' + pose.name + '"?')) {
        try {
            const response = await fetch(`${API_BASE}/api/saved_poses/${pose.id}`, {
                method: 'DELETE'
            });
            
            const data = await response.json();
            if (data.success) {
                logData(`Deleted pose: ${pose.name}`);
                await loadSavedPoses();  // Reload from server
            } else {
                alert('Failed to delete: ' + data.error);
            }
        } catch (error) {
            console.error('Delete failed:', error);
            alert('Network error: Could not delete pose');
        }
    }
}

function renderSavedPoses() {
    const listEl = document.getElementById('saved-poses-list');
    listEl.innerHTML = '';

    if (savedPoses.length === 0) {
        listEl.innerHTML = '<div class="text-center text-slate-500 text-sm py-4 italic">No saved positions yet.</div>';
        return;
    }

    savedPoses.forEach((pose, index) => {
        const row = document.createElement('div');
        row.className = 'flex items-center justify-between bg-slate-900 p-3 rounded border border-slate-700 group';
        
        const angles = Object.keys(pose.state).map(k => pose.state[k]).join(', ');
        const countInChain = chainQueue.filter(p => p.id === pose.id).length;
        const chainBadge = countInChain > 0 ? ` <span class="text-purple-400 font-bold">×${countInChain}</span>` : '';
        
        row.innerHTML = `
            <div class="flex flex-col">
                <span class="font-semibold text-slate-200">${pose.name}${chainBadge}</span>
                <span class="text-xs text-slate-500 font-mono">${angles}</span>
            </div>
            <div class="flex gap-2">
                <button onclick="addToChain(${index})" class="bg-purple-600 hover:bg-purple-500 text-white text-xs px-3 py-2 rounded transition" title="Add to chain">
                    + Chain
                </button>
                <button onclick="loadPose(${index})" class="bg-emerald-600 hover:bg-emerald-500 text-white text-xs px-3 py-2 rounded transition">Load</button>
                <button onclick="deletePose(${index})" class="bg-slate-700 hover:bg-red-600 text-slate-300 text-xs px-3 py-2 rounded transition" title="Delete">✕</button>
            </div>
        `;
        listEl.appendChild(row);
    });
}

async function emergencyStop() {
    try {
        const response = await fetch(`${API_BASE}/api/emergency_stop`, {
            method: 'POST'
        });
        const data = await response.json();
        
        if (data.success) {
            logData('!!! EMERGENCY STOP !!!', true);
            alert("EMERGENCY STOP: All servos disabled.");
        }
    } catch (error) {
        console.error('Emergency stop failed:', error);
        alert("Failed to send emergency stop!");
    }
}

// --- Data Logging ---
function logData(message, isError = false) {
    const date = new Date();
    const time = date.toLocaleTimeString();
    const dataStr = `[${time}] ${message}`;
    
    const consoleDiv = document.getElementById('data-stream');
    const newEntry = document.createElement('div');
    newEntry.innerText = dataStr;
    if (isError) newEntry.className = 'text-red-500 font-bold';
    consoleDiv.insertBefore(newEntry, consoleDiv.firstChild);
    
    if (consoleDiv.children.length > 20) {
        consoleDiv.removeChild(consoleDiv.lastChild);
    }
}

// --- Pose Chaining ---
function addToChain(index) {
    const pose = savedPoses[index];
    if (!pose) return;
    
    // Allow duplicates - just add to chain
    if (chainQueue.length >= 5) {
        alert('Maximum 5 poses in chain');
        return;
    }
    
    chainQueue.push(pose);
    logData(`Added "${pose.name}" to chain (${chainQueue.length}/5)`);
    
    renderSavedPoses();
    renderChainQueue();
}

function removeFromChain(chainIndex) {
    if (chainIndex >= 0 && chainIndex < chainQueue.length) {
        const removed = chainQueue.splice(chainIndex, 1)[0];
        logData(`Removed "${removed.name}" from chain`);
        renderSavedPoses();
        renderChainQueue();
    }
}

function clearChain() {
    chainQueue = [];
    logData('Chain cleared');
    renderSavedPoses();
    renderChainQueue();
}

function renderChainQueue() {
    const chainEl = document.getElementById('chain-queue');
    if (!chainEl) return;
    
    if (chainQueue.length === 0) {
        chainEl.innerHTML = '<div class="text-center text-slate-500 text-sm py-3 italic">No poses in chain</div>';
        return;
    }
    
    chainEl.innerHTML = chainQueue.map((pose, index) => `
        <div class="flex items-center justify-between bg-slate-900 p-2 rounded border border-purple-700">
            <div class="flex items-center gap-2">
                <span class="text-purple-400 font-bold text-sm">${index + 1}.</span>
                <span class="text-slate-200 text-sm">${pose.name}</span>
            </div>
            <button onclick="removeFromChain(${index})" class="text-slate-400 hover:text-red-400 text-sm px-2" title="Remove">✕</button>
        </div>
    `).join('');
}

async function executeChain() {
    if (!oeEnabled) {
        logData('⚠️ Motors disabled - enable motors to run chain', true);
        return;
    }
    
    if (chainQueue.length === 0) {
        logData('⚠️ Chain is empty', true);
        return;
    }
    
    if (isChainRunning) {
        logData('⚠️ Chain already running', true);
        return;
    }
    
    const anyMoving = Object.values(isMoving).some(moving => moving);
    if (anyMoving) {
        logData('⚠️ Wait for current movement to finish', true);
        return;
    }
    
    isChainRunning = true;
    updateChainButtons();
    logData(`▶ Starting chain (${chainQueue.length} poses)`);
    
    try {
        for (let i = 0; i < chainQueue.length; i++) {
            if (!isChainRunning) {
                logData('Chain stopped by user');
                break;
            }
            
            const pose = chainQueue[i];
            logData(`[${i + 1}/${chainQueue.length}] Moving to "${pose.name}"`);
            await applyState(pose.state);
            
            // Wait a bit between poses (optional delay)
            if (i < chainQueue.length - 1) {
                await new Promise(resolve => setTimeout(resolve, 500));
            }
        }
        
        logData('✓ Chain completed');
    } catch (err) {
        console.error('Chain execution error:', err);
        logData('⚠️ Chain execution failed', true);
    } finally {
        isChainRunning = false;
        updateChainButtons();
    }
}

function stopChain() {
    if (isChainRunning) {
        isChainRunning = false;
        logData('Stopping chain...');
    }
}

function updateChainButtons() {
    const runBtn = document.getElementById('run-chain-btn');
    const stopBtn = document.getElementById('stop-chain-btn');
    
    if (runBtn && stopBtn) {
        // Stop button always enabled for emergency use
        stopBtn.disabled = false;
        stopBtn.style.opacity = isChainRunning ? '1' : '0.5';
        
        // Run button disabled during execution or when conditions not met
        if (isChainRunning) {
            runBtn.disabled = true;
            runBtn.style.opacity = '0.3';
        } else {
            runBtn.disabled = !oeEnabled || chainQueue.length === 0;
            runBtn.style.opacity = (!oeEnabled || chainQueue.length === 0) ? '0.3' : '1';
        }
    }
}

// --- Initialization ---
async function init() {
    await loadLastPosition();  // Load last position first
    await checkStatus();
    await loadSavedPoses();  // Load from database
    renderChainQueue();  // Render empty chain UI
    
    // Poll status every 5 seconds
    setInterval(checkStatus, 5000);
    
    logData('Web UI initialized');
}

// Start when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
