import { Matrix4, Vector3, Euler, Quaternion, MathUtils } from 'three';
import { PoseData } from './parser';

export interface DHParameterConfig {
    d: number[];
    a: number[];
    alpha: number[];
    offsets: number[];
}

export class KinematicsService {
  // Parameters made public for reading, modified via setDHConfig
  public d = [0, 563, -180, 180, 160, 0, 0];
  public a = [0, 220, 900, 0, 1013.5, 200, 0];
  public alpha = [0, 0, 0, 0, 0, 0, 0]; 
  public jointOffsets = [-90, 0, 90, 0, 0, 45, 0];

  constructor() {}

  public getDHConfig(): DHParameterConfig {
    return {
        d: [...this.d],
        a: [...this.a],
        alpha: [...this.alpha],
        offsets: [...this.jointOffsets]
    };
  }

  public setDHConfig(config: DHParameterConfig) {
    this.d = [...config.d];
    this.a = [...config.a];
    this.alpha = [...config.alpha];
    this.jointOffsets = [...config.offsets];
  }

  private getRad(deg: number): number {
    return deg * (Math.PI / 180);
  }

  public poseToMatrix(pose: PoseData): Matrix4 {
    const m = new Matrix4();
    const translation = new Matrix4().makeTranslation(pose.x, pose.y, pose.z);
    
    // Euler Z-Y-X Convention (KUKA/Standard Industrial)
    const rotA = new Matrix4().makeRotationZ(this.getRad(pose.a));
    const rotB = new Matrix4().makeRotationY(this.getRad(pose.b));
    const rotC = new Matrix4().makeRotationX(this.getRad(pose.c));
    
    const rotation = rotA.multiply(rotB).multiply(rotC);
    m.multiply(translation).multiply(rotation);
    return m;
  }

  public calculateFlangeTarget(
    point: PoseData, 
    worldOffset: PoseData, 
    toolOffset: PoseData
  ): { position: Vector3, matrix: Matrix4 } {
    const matWorld = this.poseToMatrix(worldOffset);
    const matPoint = this.poseToMatrix(point);
    const matTool = this.poseToMatrix(toolOffset);
    
    const matTCP = matWorld.clone().multiply(matPoint);
    const matFlange = matTCP.clone().multiply(matTool.invert());
    const position = new Vector3().setFromMatrixPosition(matFlange);
    
    return { position, matrix: matFlange };
  }

  private applyTransforms(
    mat: Matrix4, 
    trans: {x: number, y: number, z: number}, 
    rot: {angleDeg: number, axis: 'x'|'y'|'z'}
  ) {
    const t = new Matrix4().makeTranslation(trans.x, trans.y, trans.z);
    mat.multiply(t);
    const r = new Matrix4();
    const rad = this.getRad(rot.angleDeg);
    if (rot.axis === 'x') r.makeRotationX(rad);
    if (rot.axis === 'y') r.makeRotationY(rad);
    if (rot.axis === 'z') r.makeRotationZ(rad);
    mat.multiply(r);
  }

  public forwardKinematicsChain(jointAnglesDeg: number[]): Matrix4[] {
    const matrices: Matrix4[] = [];
    const currentMat = new Matrix4(); 

    const jVars = [0, ...jointAnglesDeg]; 

    // --- Link 0 ---
    const m0 = currentMat.clone();
    this.applyTransforms(m0, {x:0, y:0, z:this.d[0]}, {angleDeg: jVars[0] + this.jointOffsets[0], axis: 'z'});
    this.applyTransforms(m0, {x:this.a[0], y:0, z:0}, {angleDeg: this.alpha[0], axis: 'x'});
    matrices.push(m0.clone()); 

    // --- Link 1 ---
    const m1 = m0.clone();
    this.applyTransforms(m1, {x:0, y:0, z:this.d[1]}, {angleDeg: jVars[1] + this.jointOffsets[1], axis: 'z'});
    this.applyTransforms(m1, {x:this.a[1], y:0, z:0}, {angleDeg: this.alpha[1], axis: 'x'});
    matrices.push(m1.clone());

    const m1_adj = m1.clone();
    m1_adj.multiply(new Matrix4().makeRotationX(this.getRad(90)));

    // --- Link 2 ---
    const m2 = m1_adj.clone();
    this.applyTransforms(m2, {x:0, y:0, z:this.d[2]}, {angleDeg: jVars[2] + this.jointOffsets[2], axis: 'z'});
    this.applyTransforms(m2, {x:this.a[2], y:0, z:0}, {angleDeg: this.alpha[2], axis: 'x'});
    matrices.push(m2.clone());

    // --- Link 3 ---
    const m3 = m2.clone();
    this.applyTransforms(m3, {x:0, y:0, z:this.d[3]}, {angleDeg: jVars[3] + this.jointOffsets[3], axis: 'z'});
    this.applyTransforms(m3, {x:this.a[3], y:0, z:0}, {angleDeg: this.alpha[3], axis: 'x'});
    matrices.push(m3.clone());

    const m3_adj = m3.clone();
    m3_adj.multiply(new Matrix4().makeRotationY(this.getRad(90)));
    m3_adj.multiply(new Matrix4().makeRotationZ(this.getRad(-90)));

    // --- Link 4 ---
    // NOTE: This joint rotates around local X
    const m4 = m3_adj.clone();
    this.applyTransforms(m4, {x:0, y:0, z:this.d[4]}, {angleDeg: jVars[4] + this.jointOffsets[4], axis: 'x'}); 
    this.applyTransforms(m4, {x:this.a[4], y:0, z:0}, {angleDeg: this.alpha[4], axis: 'x'});
    matrices.push(m4.clone());

    const m4_adj = m4.clone();
    m4_adj.multiply(new Matrix4().makeRotationX(this.getRad(-90)));

    // --- Link 5 ---
    const m5 = m4_adj.clone();
    this.applyTransforms(m5, {x:0, y:0, z:this.d[5]}, {angleDeg: jVars[5] + this.jointOffsets[5], axis: 'z'});
    this.applyTransforms(m5, {x:this.a[5], y:0, z:0}, {angleDeg: this.alpha[5], axis: 'x'});
    matrices.push(m5.clone());

    const m5_adj = m5.clone();
    m5_adj.multiply(new Matrix4().makeRotationX(this.getRad(-90)));
    m5_adj.multiply(new Matrix4().makeRotationY(this.getRad(90)));

    // --- Link 6 ---
    const m6 = m5_adj.clone();
    this.applyTransforms(m6, {x:0, y:0, z:this.d[6]}, {angleDeg: jVars[6] + this.jointOffsets[6], axis: 'z'});
    this.applyTransforms(m6, {x:this.a[6], y:0, z:0}, {angleDeg: this.alpha[6], axis: 'x'});
    matrices.push(m6.clone());

    // --- Tool ---
    const mTool = m6.clone();
    mTool.multiply(new Matrix4().makeTranslation(0, 0, 386.63));
    mTool.multiply(new Matrix4().makeTranslation(-124, 0, 0));
    matrices.push(mTool.clone());

    return matrices;
  }

  public getEndEffectorPose(jointAnglesDeg: number[]) {
    const chain = this.forwardKinematicsChain(jointAnglesDeg);
    const endMatrix = chain[chain.length - 1];
    
    const position = new Vector3();
    const quaternion = new Quaternion();
    const scale = new Vector3();
    
    endMatrix.decompose(position, quaternion, scale);
    const euler = new Euler().setFromQuaternion(quaternion);

    return { position, rotation: euler, matrix: endMatrix };
  }

  /**
   * Jacobian Transpose Inverse Kinematics
   * Solves for 6-DOF (Position + Orientation)
   * 
   * @param targetMatrix The target pose matrix
   * @param currentAnglesDeg Start angles
   * @returns Optimized angles
   */
  public inverseKinematics(targetMatrix: Matrix4, currentAnglesDeg: number[]): number[] {
    const newAngles = [...currentAnglesDeg];
    
    const targetPos = new Vector3();
    const targetQuat = new Quaternion();
    const s = new Vector3();
    targetMatrix.decompose(targetPos, targetQuat, s);

    // Improved accuracy settings
    const maxIterations = 200; // Increased from 50
    const learningRate = 0.5; 
    const posThreshold = 0.1; // Reduced from 1.0mm to 0.1mm
    const rotThreshold = 0.001; // Reduced from 0.01rad to 0.001rad

    for (let iter = 0; iter < maxIterations; iter++) {
      // 1. Forward Kinematics
      const chain = this.forwardKinematicsChain(newAngles);
      const endEffectorMatrix = chain[chain.length - 1];
      
      const currentPos = new Vector3();
      const currentQuat = new Quaternion();
      endEffectorMatrix.decompose(currentPos, currentQuat, s);

      // 2. Calculate Error Vector (6x1)
      const errPos = new Vector3().subVectors(targetPos, currentPos);
      
      const errQuat = targetQuat.clone().multiply(currentQuat.clone().invert());
      const errRot = new Vector3(errQuat.x, errQuat.y, errQuat.z);
      if (errQuat.w < 0) errRot.multiplyScalar(-1);
      errRot.multiplyScalar(2);

      // Check convergence
      if (errPos.length() < posThreshold && errRot.length() < rotThreshold) {
        break;
      }

      // 3. Jacobian Transpose Iteration
      for (let j = 0; j < 6; j++) {
        const pivotIdx = j; 
        const pivotMatrix = chain[pivotIdx];
        const pivotPos = new Vector3().setFromMatrixPosition(pivotMatrix);
        const pivotRot = new Quaternion().setFromRotationMatrix(pivotMatrix);

        let localAxis = new Vector3(0, 0, 1); // Default Z
        if (j === 3) { 
           // Joint 4 rotates around X
           localAxis = new Vector3(1, 0, 0);
        }
        const jointAxis = localAxis.applyQuaternion(pivotRot).normalize();
        const p_e = new Vector3().subVectors(currentPos, pivotPos);

        const J_v = new Vector3().crossVectors(jointAxis, p_e);
        const J_w = jointAxis.clone();

        // Higher weight on Position to ensure path following
        const contribution = J_v.dot(errPos) + J_w.dot(errRot) * 100; 

        newAngles[j] += contribution * learningRate * 0.00005;
      }
    }
    
    // Normalize angles
    return newAngles.map(a => {
        let angle = a % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    });
  }
}

export const kinematics = new KinematicsService();
