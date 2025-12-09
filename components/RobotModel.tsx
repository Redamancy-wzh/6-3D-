import React, { useMemo } from 'react';
import { Matrix4, Vector3, Quaternion } from 'three';
import { kinematics, DHParameterConfig } from '../services/kinematics';

interface RobotModelProps {
  jointAngles: number[]; 
  showAxes?: boolean;
  dhConfig?: DHParameterConfig; // Add dependency for useMemo
}

const colors = [
  "#ff4500", // Link 0 
  "#a9a9a9", // Link 1 
  "#ff4500", // Link 2 
  "#a9a9a9", // Link 3 
  "#ff4500", // Link 4 
  "#a9a9a9", // Link 5 
  "#ff4500", // Link 6 
  "#00ffff", // Tool 
];

const Segment: React.FC<{ 
  startMat: Matrix4, 
  endMat: Matrix4,
  color: string,
  thickness: number
}> = ({ startMat, endMat, color, thickness }) => {
  const startPos = new Vector3().setFromMatrixPosition(startMat);
  const endPos = new Vector3().setFromMatrixPosition(endMat);
  
  const distance = startPos.distanceTo(endPos);
  
  if (distance < 1) return null; 

  const midPos = new Vector3().addVectors(startPos, endPos).multiplyScalar(0.5);
  const up = new Vector3(0, 1, 0);
  const direction = new Vector3().subVectors(endPos, startPos).normalize();
  const quaternion = new Quaternion().setFromUnitVectors(up, direction);

  return (
    <mesh position={midPos} quaternion={quaternion}>
      <cylinderGeometry args={[thickness, thickness, distance, 16]} />
      <meshStandardMaterial color={color} />
    </mesh>
  );
};

export const RobotModel: React.FC<RobotModelProps> = ({ jointAngles, showAxes = false, dhConfig }) => {
  const matrices = useMemo(() => {
    // We add dhConfig to the dependency array, so if parameters change, 
    // the model is rebuilt using the new internal state of the kinematics service.
    return kinematics.forwardKinematicsChain(jointAngles);
  }, [jointAngles, dhConfig]);

  return (
    <group>
      {/* Robot Base Platform - Aligned with Z axis */}
      <mesh position={[0, 0, -50]} rotation={[Math.PI/2, 0, 0]}>
        <cylinderGeometry args={[250, 300, 100, 32]} />
        <meshStandardMaterial color="#333" />
      </mesh>

      {/* Visual Joints/Nodes */}
      {matrices.map((mat, i) => {
        const pos = new Vector3().setFromMatrixPosition(mat);
        const quat = new Quaternion().setFromRotationMatrix(mat);
        const isTool = i === matrices.length - 1;
        
        return (
          <group key={`node-${i}`} position={pos} quaternion={quat}>
            {/* Cylinders represent the joints. Rotated to align with local X or Z? 
                Code rotates 90 deg X, making them perpendicular to local Z. 
                This usually represents the rotational axis or body of motor. */}
            <mesh rotation={[Math.PI/2, 0, 0]}>
              <cylinderGeometry args={[isTool ? 20 : 70, isTool ? 20 : 70, 120, 32]} />
              <meshStandardMaterial color={colors[i] || "#ffffff"} />
            </mesh>
            {showAxes && <axesHelper args={[250]} />}
          </group>
        );
      })}

      {/* Segments connecting the frames */}
      {matrices.map((mat, i) => {
        if (i === 0) return null;
        const prevMat = matrices[i-1];
        
        return (
          <Segment 
            key={`seg-${i}`}
            startMat={prevMat}
            endMat={mat}
            color={colors[i-1] || "#888"}
            thickness={isFinite(i) && i > 4 ? 40 : 60}
          />
        );
      })}
    </group>
  );
};
