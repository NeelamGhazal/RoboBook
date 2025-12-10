import React from 'react';

/**
 * Static SVG fallback for 3D animated cube
 * Used when:
 * - User is on mobile (< 768px)
 * - User prefers reduced motion
 * - WebGL is not supported
 *
 * Displays a static cube with neon cyan gradient to match the animated version
 */
export const FallbackSVG: React.FC = () => (
  <svg
    width="400"
    height="400"
    viewBox="0 0 400 400"
    xmlns="http://www.w3.org/2000/svg"
    role="img"
    aria-label="Animated 3D cube visualization"
    style={{ maxWidth: '100%', height: 'auto' }}
  >
    <defs>
      <linearGradient id="neonGlow" x1="0%" y1="0%" x2="100%" y2="100%">
        <stop offset="0%" stopColor="#00F0FF" stopOpacity="0.8" />
        <stop offset="100%" stopColor="#FF2A6D" stopOpacity="0.6" />
      </linearGradient>
    </defs>

    {/* Cube outline with neon edges */}
    <g transform="translate(200, 200) rotate(20)">
      {/* Front face */}
      <path
        d="M-80,-80 L80,-80 L80,80 L-80,80 Z"
        fill="none"
        stroke="url(#neonGlow)"
        strokeWidth="3"
        opacity="0.8"
      />
      {/* Back face (offset for 3D effect) */}
      <path
        d="M-60,-100 L100,-100 L100,60 L-60,60 Z"
        fill="none"
        stroke="#00F0FF"
        strokeWidth="2"
        opacity="0.6"
      />
      {/* Connecting edges */}
      <path
        d="M-80,-80 L-60,-100 M80,-80 L100,-100 M80,80 L100,60 M-80,80 L-60,60"
        stroke="#00F0FF"
        strokeWidth="2"
        opacity="0.7"
      />
    </g>
  </svg>
);
