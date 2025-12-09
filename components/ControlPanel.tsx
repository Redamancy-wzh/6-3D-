import React, { useState } from 'react';
import { Vector3 } from 'three';
import { DHParameterConfig } from '../services/kinematics';

interface ControlPanelProps {
  jointAngles: number[];
  setJointAngles: (angles: number[]) => void;
  endEffectorPose: { position: Vector3, rotation: any };
  onRunTrajectory: (type: 'line' | 's-curve' | 'spiral') => void;
  onSolveIK: (target: Vector3) => void;
  showAxes: boolean;
  setShowAxes: (v: boolean) => void;
  showTrail: boolean;
  setShowTrail: (v: boolean) => void;
  showGrid: boolean;
  setShowGrid: (v: boolean) => void;
  onFilesSelected?: (files: FileList) => void;
  
  // DH Props
  dhParams: DHParameterConfig;
  setDhParams: (config: DHParameterConfig) => void;
}

export const ControlPanel: React.FC<ControlPanelProps> = ({
  jointAngles,
  setJointAngles,
  endEffectorPose,
  onRunTrajectory,
  onSolveIK,
  showAxes,
  setShowAxes,
  showTrail,
  setShowTrail,
  showGrid,
  setShowGrid,
  onFilesSelected,
  dhParams,
  setDhParams
}) => {
  const [activeTab, setActiveTab] = useState<'control' | 'dh'>('control');
  const [ikTarget, setIkTarget] = useState({ x: 1000, y: 0, z: 1000 });

  const handleSliderChange = (index: number, value: string) => {
    const newAngles = [...jointAngles];
    newAngles[index] = parseFloat(value);
    setJointAngles(newAngles);
  };

  const handleIKSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSolveIK(new Vector3(ikTarget.x, ikTarget.y, ikTarget.z));
  };

  // Helper to update a specific DH value
  const updateDH = (arrayName: keyof DHParameterConfig, index: number, value: string) => {
    const num = parseFloat(value);
    if (isNaN(num)) return;
    
    const newConfig = { ...dhParams };
    newConfig[arrayName] = [...dhParams[arrayName]];
    newConfig[arrayName][index] = num;
    setDhParams(newConfig);
  };

  return (
    <div className="absolute right-0 top-0 bottom-0 w-80 bg-gray-900/95 backdrop-blur-md border-l border-gray-700 overflow-hidden shadow-2xl z-10 flex flex-col font-mono">
      
      {/* Header & Tabs */}
      <div className="p-4 pb-0 bg-gray-800 border-b border-gray-700 shrink-0">
        <h2 className="text-xl font-bold text-orange-500 mb-2">ER50A Sim</h2>
        
        <div className="flex gap-1 mt-2">
            <button 
                onClick={() => setActiveTab('control')}
                className={`flex-1 py-2 text-xs font-semibold rounded-t-lg transition-colors ${
                    activeTab === 'control' 
                    ? 'bg-gray-700 text-white border-t border-x border-gray-600' 
                    : 'bg-transparent text-gray-500 hover:text-gray-300 hover:bg-gray-800'
                }`}
            >
                Controls
            </button>
            <button 
                onClick={() => setActiveTab('dh')}
                className={`flex-1 py-2 text-xs font-semibold rounded-t-lg transition-colors ${
                    activeTab === 'dh' 
                    ? 'bg-gray-700 text-white border-t border-x border-gray-600' 
                    : 'bg-transparent text-gray-500 hover:text-gray-300 hover:bg-gray-800'
                }`}
            >
                DH Params
            </button>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto p-4 text-sm scrollbar-thin scrollbar-thumb-gray-600">
        
        {/* --- CONTROL TAB --- */}
        {activeTab === 'control' && (
            <div className="flex flex-col gap-6">
                
                {/* File Import */}
                <div className="bg-gray-800 p-3 rounded border border-gray-600">
                    <h3 className="text-white font-semibold mb-2">Import Trajectory</h3>
                    <p className="text-[10px] text-gray-400 mb-2">Select _globalvars.txt and commontest.txt</p>
                    <input 
                        type="file" 
                        multiple 
                        accept=".txt"
                        onChange={(e) => e.target.files && onFilesSelected && onFilesSelected(e.target.files)}
                        className="w-full text-xs text-gray-300 file:mr-2 file:py-1 file:px-2 file:rounded file:border-0 file:text-xs file:bg-blue-700 file:text-white hover:file:bg-blue-600"
                    />
                </div>

                {/* Joint Controls */}
                <div className="space-y-4">
                    <h3 className="text-white font-semibold border-b border-gray-700 pb-2">Joints (Deg)</h3>
                    {jointAngles.map((angle, i) => (
                    <div key={i} className="flex flex-col gap-1">
                        <div className="flex justify-between text-gray-400 text-xs">
                        <span>J{i + 1}</span>
                        <span>{angle.toFixed(1)}°</span>
                        </div>
                        <input
                        type="range"
                        min={-180}
                        max={180}
                        step={0.1}
                        value={angle}
                        onChange={(e) => handleSliderChange(i, e.target.value)}
                        className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-orange-500 hover:accent-orange-400"
                        />
                    </div>
                    ))}
                    <button 
                        onClick={() => setJointAngles([0,0,0,0,0,0])}
                        className="w-full py-1.5 bg-gray-700 hover:bg-gray-600 rounded text-xs text-white transition-colors"
                    >
                        Reset Home
                    </button>
                </div>

                {/* End Effector Output */}
                <div className="bg-gray-800 p-3 rounded border border-gray-700">
                    <h3 className="text-orange-400 font-semibold mb-2">Flange Pose (mm)</h3>
                    <div className="grid grid-cols-2 gap-2 text-xs">
                    <div>
                        <span className="text-gray-500 block">Position (X,Y,Z)</span>
                        <span className="font-mono text-white">
                        {endEffectorPose.position.x.toFixed(0)}, <br/>
                        {endEffectorPose.position.y.toFixed(0)}, <br/>
                        {endEffectorPose.position.z.toFixed(0)}
                        </span>
                    </div>
                    <div>
                        <span className="text-gray-500 block">Euler (Deg)</span>
                        <span className="font-mono text-white">
                        {(endEffectorPose.rotation.x * 180 / Math.PI).toFixed(1)}, <br/>
                        {(endEffectorPose.rotation.y * 180 / Math.PI).toFixed(1)}, <br/>
                        {(endEffectorPose.rotation.z * 180 / Math.PI).toFixed(1)}
                        </span>
                    </div>
                    </div>
                </div>

                {/* Inverse Kinematics */}
                <div className="space-y-3">
                    <h3 className="text-white font-semibold border-b border-gray-700 pb-2">Inverse Kinematics</h3>
                    <form onSubmit={handleIKSubmit} className="grid grid-cols-3 gap-2">
                        <div>
                            <label className="text-[10px] text-gray-400 block">X</label>
                            <input 
                                type="number" step="10"
                                value={ikTarget.x} 
                                onChange={e => setIkTarget({...ikTarget, x: parseFloat(e.target.value)})} 
                                className="w-full bg-gray-800 border border-gray-600 rounded px-1 py-1 text-white"
                            />
                        </div>
                        <div>
                            <label className="text-[10px] text-gray-400 block">Y</label>
                            <input 
                                type="number" step="10"
                                value={ikTarget.y} 
                                onChange={e => setIkTarget({...ikTarget, y: parseFloat(e.target.value)})} 
                                className="w-full bg-gray-800 border border-gray-600 rounded px-1 py-1 text-white"
                            />
                        </div>
                        <div>
                            <label className="text-[10px] text-gray-400 block">Z</label>
                            <input 
                                type="number" step="10"
                                value={ikTarget.z} 
                                onChange={e => setIkTarget({...ikTarget, z: parseFloat(e.target.value)})} 
                                className="w-full bg-gray-800 border border-gray-600 rounded px-1 py-1 text-white"
                            />
                        </div>
                        <button 
                            type="submit"
                            className="col-span-3 py-2 bg-blue-600 hover:bg-blue-500 text-white rounded font-medium transition-colors text-xs"
                        >
                            Move (Iterative IK)
                        </button>
                    </form>
                </div>

                {/* Trajectory */}
                <div className="space-y-3">
                    <h3 className="text-white font-semibold border-b border-gray-700 pb-2">Demo Trajectories</h3>
                    <div className="grid grid-cols-1 gap-2">
                        <button 
                            onClick={() => onRunTrajectory('line')}
                            className="py-2 bg-green-700 hover:bg-green-600 text-white rounded text-xs"
                        >
                            Linear (Current to Home)
                        </button>
                        <button 
                            onClick={() => onRunTrajectory('s-curve')}
                            className="py-2 bg-purple-700 hover:bg-purple-600 text-white rounded text-xs"
                        >
                            S-Curve (Planar)
                        </button>
                        <button 
                            onClick={() => onRunTrajectory('spiral')}
                            className="py-2 bg-pink-700 hover:bg-pink-600 text-white rounded text-xs"
                        >
                            Spiral (Vertical)
                        </button>
                    </div>
                    
                    <div className="space-y-2 pt-2 border-t border-gray-700">
                        <label className="flex items-center gap-2 cursor-pointer">
                            <input type="checkbox" checked={showGrid} onChange={e => setShowGrid(e.target.checked)} className="rounded bg-gray-700 border-gray-600 text-orange-500" />
                            <span className="text-gray-300">Show Grid</span>
                        </label>
                        <label className="flex items-center gap-2 cursor-pointer">
                            <input type="checkbox" checked={showAxes} onChange={e => setShowAxes(e.target.checked)} className="rounded bg-gray-700 border-gray-600 text-orange-500" />
                            <span className="text-gray-300">Show Coordinates</span>
                        </label>
                        <label className="flex items-center gap-2 cursor-pointer">
                            <input type="checkbox" checked={showTrail} onChange={e => setShowTrail(e.target.checked)} className="rounded bg-gray-700 border-gray-600 text-orange-500" />
                            <span className="text-gray-300">Show Trail</span>
                        </label>
                    </div>
                </div>
            </div>
        )}

        {/* --- DH PARAMS TAB --- */}
        {activeTab === 'dh' && (
            <div className="flex flex-col gap-4">
                <p className="text-[10px] text-gray-400 mb-2">Adjust DH parameters. Visuals update in real-time.</p>
                
                {dhParams.d.map((_, i) => (
                    <div key={i} className="bg-gray-800 p-2 rounded border border-gray-600 mb-2">
                        <div className="flex justify-between items-center mb-1">
                            <span className="text-orange-400 font-bold text-xs">Link {i}</span>
                        </div>
                        <div className="grid grid-cols-2 gap-2 text-xs">
                            <div className="flex flex-col">
                                <label className="text-gray-500">d (mm)</label>
                                <input 
                                    type="number" 
                                    value={dhParams.d[i]} 
                                    onChange={(e) => updateDH('d', i, e.target.value)}
                                    className="bg-gray-700 text-white border border-gray-600 rounded px-1 py-0.5"
                                />
                            </div>
                            <div className="flex flex-col">
                                <label className="text-gray-500">a (mm)</label>
                                <input 
                                    type="number" 
                                    value={dhParams.a[i]} 
                                    onChange={(e) => updateDH('a', i, e.target.value)}
                                    className="bg-gray-700 text-white border border-gray-600 rounded px-1 py-0.5"
                                />
                            </div>
                            <div className="flex flex-col">
                                <label className="text-gray-500">alpha (deg)</label>
                                <input 
                                    type="number" 
                                    value={dhParams.alpha[i]} 
                                    onChange={(e) => updateDH('alpha', i, e.target.value)}
                                    className="bg-gray-700 text-white border border-gray-600 rounded px-1 py-0.5"
                                />
                            </div>
                            <div className="flex flex-col">
                                <label className="text-gray-500">Offset (deg)</label>
                                <input 
                                    type="number" 
                                    value={dhParams.offsets[i]} 
                                    onChange={(e) => updateDH('offsets', i, e.target.value)}
                                    className="bg-gray-700 text-white border border-gray-600 rounded px-1 py-0.5"
                                />
                            </div>
                        </div>
                    </div>
                ))}
            </div>
        )}
      </div>
    </div>
  );
};
