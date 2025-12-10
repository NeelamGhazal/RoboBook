import { useState, useEffect } from 'react';

/**
 * Custom hook to detect WebGL support in the browser
 * Required for Three.js 3D rendering
 * @returns boolean indicating if WebGL is supported
 *
 * @example
 * const isWebGLSupported = useWebGLSupport();
 * if (isWebGLSupported) {
 *   // Render Three.js scene
 * } else {
 *   // Show fallback SVG
 * }
 */
export function useWebGLSupport(): boolean {
  const [isSupported, setIsSupported] = useState(true);

  useEffect(() => {
    try {
      const canvas = document.createElement('canvas');
      const gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
      setIsSupported(!!gl);
    } catch {
      setIsSupported(false);
    }
  }, []);

  return isSupported;
}
