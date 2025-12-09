import React, { useState, useEffect, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, PerspectiveCamera, Environment, Line, Text } from '@react-three/drei';
import { Vector3, Matrix4, Quaternion } from 'three';
import { kinematics, DHParameterConfig } from './services/kinematics';
import { ParserService, RobotGlobalConfig } from './services/parser';
import { RobotModel } from './components/RobotModel';
import { ControlPanel } from './components/ControlPanel';

const TrajectoryLines: React.FC<{ 
    points: Vector3[], 
    visible: boolean 
}> = ({ points, visible }) => {
    if (!visible || points.length < 2) return null;
    return (
        <Line 
            points={points} 
            color="#22d3ee" 
            lineWidth={3} 
            opacity={0.8}
            transparent
        />
    );
};

const TrajectoryPlayer = ({ queue, currentAngles, setAngles, onFinished }: any) => {
    useFrame((state, delta) => {
        if (queue.length === 0) return;

        const target = queue[0];
        let reached = true;
        // Increased speed for smoother playback of dense point files
        const speed = 120.0 * delta; // Degrees per frame approx

        const nextAngles = currentAngles.map((curr: number, i: number) => {
            const diff = target[i] - curr;
            if (Math.abs(diff) > 0.1) {
                reached = false;
                return curr + Math.sign(diff) * Math.min(Math.abs(diff), speed);
            }
            return target[i];
        });

        setAngles(nextAngles);

        if (reached) {
            queue.shift(); 
            if (queue.length === 0) onFinished();
        }
    });
    return null;
};

const GlobalAxesHelper = () => {
    return (
        <group position={[0, 0, 0]}>
            <axesHelper args={[1000]} />
            <Text position={[1100, 0, 0]} fontSize={100} color="red">X</Text>
            <Text position={[0, 1100, 0]} fontSize={100} color="green">Y</Text>
            <Text position={[0, 0, 1100]} fontSize={100} color="blue">Z</Text>
        </group>
    );
};

const WorldFrameHelper: React.FC<{ config: RobotGlobalConfig | null }> = ({ config }) => {
    if (!config) return null;
    const pos = new Vector3(config.world.x, config.world.y, config.world.z);
    
    const mat = kinematics.poseToMatrix(config.world);
    const quat = new Quaternion().setFromRotationMatrix(mat);

    return (
        <group position={pos} quaternion={quat}>
            <axesHelper args={[500]} />
            <mesh position={[0,0,0]}>
                <sphereGeometry args={[30]} />
                <meshStandardMaterial color="yellow" />
            </mesh>
        </group>
    );
};

const App: React.FC = () => {
  const [jointAngles, setJointAngles] = useState<number[]>([0, 0, 0, 0, 0, 0]);
  
  // State for DH Parameters (initialized from service defaults)
  const [dhParams, setDhParams] = useState<DHParameterConfig>(kinematics.getDHConfig());

  // Visual Options
  const [showAxes, setShowAxes] = useState(true);
  const [showTrail, setShowTrail] = useState(true);
  const [showGrid, setShowGrid] = useState(true);
  
  // Sync React state to Kinematics Service whenever it changes
  useEffect(() => {
    kinematics.setDHConfig(dhParams);
  }, [dhParams]);

  const endEffectorPose = kinematics.getEndEffectorPose(jointAngles);
  
  const [trajectory, setTrajectory] = useState<Vector3[]>([]);
  const lastPointRef = useRef<Vector3 | null>(null);

  // Loaded Data
  const [globalConfig, setGlobalConfig] = useState<RobotGlobalConfig | null>(null);
  const [importedTrajectory, setImportedTrajectory] = useState<Vector3[]>([]);

  // Queue for animation
  const [angleQueue, setAngleQueue] = useState<number[][]>([]);

  useEffect(() => {
    if (!showTrail) {
        if (trajectory.length > 0) setTrajectory([]);
        return;
    }
    const currentPos = endEffectorPose.position.clone();
    // Optimize trail: only add if moved > 5mm
    if (!lastPointRef.current || lastPointRef.current.distanceTo(currentPos) > 5) { 
        setTrajectory(prev => {
            const next = [...prev, currentPos];
            if (next.length > 3000) next.shift(); 
            return next;
        });
        lastPointRef.current = currentPos;
    }
  }, [endEffectorPose, showTrail]);

  const handleSolveIK = (target: Vector3) => {
    setAngleQueue([]); 
    const currentPose = kinematics.getEndEffectorPose(jointAngles);
    const targetMatrix = new Matrix4().compose(target, new Quaternion().setFromEuler(currentPose.rotation), new Vector3(1,1,1));
    
    const solution = kinematics.inverseKinematics(targetMatrix, jointAngles);
    setAngleQueue([solution]);
  };

  const handleFilesLoaded = (files: FileList) => {
    let varsText = "";
    let trajText = "";
    let readCount = 0;

    const processFiles = () => {
        if (readCount < files.length) return;
        
        let config = globalConfig || { 
            world: { x:0, y:0, z:0, a:0, b:0, c:0 }, 
            tool: { x:0, y:0, z:0, a:0, b:0, c:0 } 
        };

        if (varsText) {
            config = ParserService.parseGlobalVars(varsText);
            setGlobalConfig(config);
            console.log("Parsed Global Vars:", config);
        }

        if (trajText) {
            const points = ParserService.parseTrajectory(trajText);
            console.log(`Parsed ${points.length} points.`);
            
            const baseTargets: Vector3[] = [];
            const solutions: number[][] = [];
            let currentSeed = [...jointAngles];

            // Changed from dynamic step to 1 to ensure every point is solved and played back
            // This prevents "messy" motion by strictly adhering to the file trajectory
            const step = 1;

            for (let i = 0; i < points.length; i+=step) {
                const pt = points[i];
                const result = kinematics.calculateFlangeTarget(pt, config.world, config.tool);
                baseTargets.push(result.position);
                
                // Solve IK with better precision logic implicitly via service update
                const sol = kinematics.inverseKinematics(result.matrix, currentSeed);
                solutions.push(sol);
                currentSeed = sol; 
            }
            
            setImportedTrajectory(baseTargets);
            setAngleQueue(solutions);
        }
    };

    Array.from(files).forEach(file => {
        const reader = new FileReader();
        reader.onload = (e) => {
            const content = e.target?.result as string;
            if (file.name.includes("_globalvars.txt")) varsText = content;
            if (file.name.includes("commontest.txt")) trajText = content;
            readCount++;
            if (readCount === files.length) processFiles();
        };
        reader.readAsText(file);
    });
  };

  const generateTrajectoryPoints = (type: 'line' | 's-curve' | 'spiral') => {
    setAngleQueue([]);
    setTrajectory([]); 
    setImportedTrajectory([]);
    
    const startPose = kinematics.getEndEffectorPose(jointAngles);
    const startPos = startPose.position.clone();
    const startQuat = new Quaternion().setFromEuler(startPose.rotation);

    const steps = 60;
    const solutions: number[][] = [];
    let currentSeed = [...jointAngles];

    // Helper to solve and push
    const addPt = (pos: Vector3) => {
        const mat = new Matrix4().compose(pos, startQuat, new Vector3(1,1,1));
        const sol = kinematics.inverseKinematics(mat, currentSeed);
        solutions.push(sol);
        currentSeed = sol;
    }

    if (type === 'line') {
        const endPos = new Vector3(startPos.x + 500, startPos.y, startPos.z); 
        for (let i = 0; i <= steps; i++) {
            addPt(new Vector3().lerpVectors(startPos, endPos, i / steps));
        }
    } else if (type === 's-curve') {
        const z = startPos.z;
        const centerX = startPos.x;
        const centerY = startPos.y;
        for (let i = 0; i <= steps * 2; i++) {
            const t = i / 20; 
            const x = centerX + Math.sin(t) * 200;
            const y = centerY + i * 10;
            addPt(new Vector3(x, y, z));
        }
    } else if (type === 'spiral') {
        const centerX = 1200;
        const centerY = 0;
        const startZ = 800;
        for (let i = 0; i <= steps * 3; i++) {
            const angle = i * 0.1;
            const radius = 200 + i * 2;
            const x = centerX + Math.cos(angle) * radius;
            const y = centerY + Math.sin(angle) * radius;
            const z = startZ + i * 5;
            addPt(new Vector3(x, y, z));
        }
    }
    
    setAngleQueue(solutions);
  };

  return (
    <div className="relative w-full h-full bg-slate-900">
      <Canvas shadows dpr={[1, 2]} camera={{ position: [3000, 2000, 3000], fov: 45, near: 10, far: 100000 }}>
        
        {/* Global World Axes */}
        <GlobalAxesHelper />

        {/* Rotate the Robot Coordinate System (Z-up) to align with World Y-up */}
        <group rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
            {/* Pass dhConfig to RobotModel to trigger re-render on param change */}
            <RobotModel jointAngles={jointAngles} showAxes={showAxes} dhConfig={dhParams} />
            
            <TrajectoryLines points={trajectory} visible={showTrail} />
            
            {importedTrajectory.length > 0 && showTrail && (
               <Line points={importedTrajectory} color="#00ff00" lineWidth={1} transparent opacity={0.5} />
            )}

            <WorldFrameHelper config={globalConfig} />
            
            {/* Robot Base Frame Helper */}
            {showAxes && <axesHelper args={[1000]} />}
        </group>

        {showGrid && (
            <Grid 
                position={[0, 0, 0]} 
                args={[10000, 10000]} 
                cellSize={500} 
                cellThickness={1} 
                cellColor="#4b5563" 
                sectionSize={2500} 
                sectionThickness={1.5} 
                sectionColor="#9ca3af" 
                fadeDistance={20000} 
            />
        )}

        <TrajectoryPlayer 
            queue={angleQueue} 
            currentAngles={jointAngles} 
            setAngles={setJointAngles} 
            onFinished={() => {}}
        />

        <OrbitControls makeDefault target={[0, 800, 0]} minDistance={100} maxDistance={20000} />
        
        <Environment preset="city" />
        <ambientLight intensity={1.5} />
        <directionalLight position={[5000, 10000, 5000]} intensity={2} castShadow />
        <pointLight position={[-3000, 5000, -3000]} intensity={1} />
      </Canvas>

      <ControlPanel 
        jointAngles={jointAngles}
        setJointAngles={(angles) => { setAngleQueue([]); setJointAngles(angles); }}
        endEffectorPose={endEffectorPose}
        onRunTrajectory={generateTrajectoryPoints}
        onSolveIK={handleSolveIK}
        showAxes={showAxes}
        setShowAxes={setShowAxes}
        showTrail={showTrail}
        setShowTrail={setShowTrail}
        showGrid={showGrid}
        setShowGrid={setShowGrid}
        onFilesSelected={handleFilesLoaded}
        dhParams={dhParams}
        setDhParams={setDhParams}
      />
    </div>
  );
};

export default App;
