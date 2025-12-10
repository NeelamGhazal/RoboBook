/**
 * Three.js Scene Component Interface
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Landing page animated 3D cube with Ocean Sapphire gradient edges
 * Location: website/src/components/ThreeScene.tsx
 * SSR Safety: Must be wrapped in Docusaurus <BrowserOnly> component
 */

import type * as THREE from 'three';

/**
 * ThreeScene Component Props
 *
 * Configurable parameters for the animated Three.js scene
 */
export interface ThreeSceneProps {
  /**
   * Canvas width in pixels
   * @default Container width (100% of parent)
   * @example 800
   */
  width?: number;

  /**
   * Canvas height in pixels
   * @default 400
   * @example 600
   */
  height?: number;

  /**
   * Rotation speed in radians per frame
   * @default 0.01
   * @range 0.001 - 0.05
   * @example 0.02 (faster rotation)
   */
  rotationSpeed?: number;

  /**
   * Enable wireframe mode for cube
   * @default true
   * @description Wireframe shows Ocean Sapphire gradient edges clearly
   */
  wireframe?: boolean;

  /**
   * Geometry type: cube or sphere
   * @default 'cube'
   * @description 'sphere' uses IcosahedronGeometry for rounder shape
   */
  geometryType?: 'cube' | 'sphere';

  /**
   * Starting edge color (Ocean Sapphire accent cyan)
   * @default '#0096ff'
   * @description Must match CSS variable --ocean-accent-cyan
   */
  edgeColorStart?: string;

  /**
   * Ending edge color (Ocean Sapphire soft cyan)
   * @default '#00d4ff'
   * @description Must match CSS variable --ocean-soft-cyan
   */
  edgeColorEnd?: string;

  /**
   * Enable anti-aliasing for smoother edges
   * @default true
   * @performance May reduce FPS on low-end devices
   */
  antialias?: boolean;

  /**
   * Camera Z-axis distance from geometry
   * @default 5
   * @range 2 - 10
   * @description Larger values = smaller visual size
   */
  cameraDistance?: number;
}

/**
 * Three.js Scene State (Internal)
 *
 * Managed state for the animation loop
 */
export interface ThreeSceneState {
  scene: THREE.Scene | null;
  camera: THREE.PerspectiveCamera | null;
  renderer: THREE.WebGLRenderer | null;
  mesh: THREE.Mesh | null;
  animationFrameId: number | null;
}

/**
 * Three.js Scene Lifecycle Hooks
 *
 * Methods for scene management
 */
export interface ThreeSceneHooks {
  /**
   * Initialize Three.js scene, camera, renderer, and geometry
   * Called once in useEffect on component mount
   */
  initScene: () => void;

  /**
   * Animation loop using requestAnimationFrame
   * Rotates geometry and renders scene at 60fps
   */
  animate: () => void;

  /**
   * Cleanup Three.js resources on component unmount
   * Disposes renderer, removes canvas, cancels animation frame
   */
  cleanup: () => void;

  /**
   * Handle window resize events
   * Updates camera aspect ratio and renderer size
   */
  handleResize: () => void;
}

/**
 * Ocean Sapphire Color Constants
 *
 * Matches CSS variables from OceanSapphireVariables.css
 */
export const OceanSapphireColors = {
  ACCENT_CYAN: '#0096ff',
  SOFT_CYAN: '#00d4ff',
  PRIMARY_DEEP: '#001529',
  MID_BLUE: '#002140',
  LIGHT_BLUE: '#003a6d',
} as const;

/**
 * Performance Targets
 *
 * Ocean Sapphire Three.js performance requirements
 */
export const PerformanceTargets = {
  TARGET_FPS: 60,
  MIN_FPS: 30, // Acceptable minimum on low-end devices
  MAX_GEOMETRY_COMPLEXITY: 1000, // Max vertices/faces
  MAX_RENDER_TIME_MS: 16.67, // 60fps = 16.67ms per frame
} as const;

/**
 * Browser Compatibility
 *
 * Three.js scene requires WebGL support
 */
export const BrowserRequirements = {
  WEBGL_REQUIRED: true,
  MIN_CHROME_VERSION: 90,
  MIN_FIREFOX_VERSION: 88,
  MIN_SAFARI_VERSION: 14,
} as const;

/**
 * Usage Example:
 *
 * ```tsx
 * // website/src/pages/index.tsx
 * import BrowserOnly from '@docusaurus/BrowserOnly';
 *
 * export default function LandingPage() {
 *   return (
 *     <div className="ocean-landing">
 *       <BrowserOnly fallback={<div>Loading 3D scene...</div>}>
 *         {() => {
 *           const ThreeScene = require('../components/ThreeScene').default;
 *           return (
 *             <ThreeScene
 *               width={800}
 *               height={400}
 *               rotationSpeed={0.01}
 *               wireframe={true}
 *               geometryType="cube"
 *             />
 *           );
 *         }}
 *       </BrowserOnly>
 *     </div>
 *   );
 * }
 * ```
 *
 * Validation Checklist:
 * - [ ] Component wrapped in <BrowserOnly> to prevent SSR errors
 * - [ ] Cleanup function disposes renderer and cancels animation frame
 * - [ ] Edge colors match CSS variables (--ocean-accent-cyan, --ocean-soft-cyan)
 * - [ ] Maintains 60fps on mid-range devices (iPhone 12, Galaxy S21)
 * - [ ] Fallback content displayed during SSR phase
 */
