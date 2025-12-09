import { Vector3, Euler, MathUtils } from 'three';

export interface PoseData {
  x: number;
  y: number;
  z: number;
  a: number; // Rotation A (usually Z)
  b: number; // Rotation B (usually Y)
  c: number; // Rotation C (usually X)
}

export interface RobotGlobalConfig {
  world: PoseData;
  tool: PoseData;
}

export class ParserService {
  
  // Default values
  private static defaultPose: PoseData = { x: 0, y: 0, z: 0, a: 0, b: 0, c: 0 };

  private static parseKeyValue(text: string, key: string): number | null {
    // Regex to find "key := number" or "key := -number"
    // Case insensitive, handles spaces
    const regex = new RegExp(`${key}\\s*:=\\s*([-\\d.]+)`, 'i');
    const match = text.match(regex);
    return match ? parseFloat(match[1]) : null;
  }

  private static parseLineToPose(line: string): PoseData {
    return {
      x: this.parseKeyValue(line, 'x') ?? 0,
      y: this.parseKeyValue(line, 'y') ?? 0,
      z: this.parseKeyValue(line, 'z') ?? 0,
      a: this.parseKeyValue(line, 'a') ?? 0,
      b: this.parseKeyValue(line, 'b') ?? 0,
      c: this.parseKeyValue(line, 'c') ?? 0,
    };
  }

  public static parseGlobalVars(text: string): RobotGlobalConfig {
    const config: RobotGlobalConfig = {
      world: { ...this.defaultPose },
      tool: { ...this.defaultPose }
    };

    const lines = text.split('\n');
    for (const line of lines) {
      if (line.trim().startsWith('world1')) {
        config.world = this.parseLineToPose(line);
      } else if (line.trim().startsWith('tool1')) {
        config.tool = this.parseLineToPose(line);
      }
    }
    
    return config;
  }

  public static parseTrajectory(text: string): PoseData[] {
    const points: PoseData[] = [];
    const lines = text.split('\n');
    
    for (const line of lines) {
      // Look for lines defining CARTPOS, typically starting with "cp..."
      if (line.includes('CARTPOS') && line.includes('x :=')) {
        points.push(this.parseLineToPose(line));
      }
    }
    
    return points;
  }
}
