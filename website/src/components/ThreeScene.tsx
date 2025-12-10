/**
 * ThreeScene Component
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Landing page animated humanoid wireframe with Ocean Sapphire details
 * Tasks: T018-T023
 *
 * IMPORTANT: This component must be wrapped in <BrowserOnly> to prevent SSR errors
 */

import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { ThreeSceneProps } from '../../../specs/003-ocean-sapphire-theme/contracts/ThreeScene.interface';

const ThreeScene: React.FC<ThreeSceneProps> = ({
  width,
  height = 400,
  rotationSpeed = 0.01,
  wireframe = true,
  geometryType = 'humanoid',
  edgeColorStart = '#0096ff', // --ocean-accent-cyan
  edgeColorEnd = '#00d4ff', // --ocean-soft-cyan
  antialias = true,
  cameraDistance = 5,
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const groupRef = useRef<THREE.Group | null>(null);
  const animationFrameIdRef = useRef<number | null>(null);
  const clockRef = useRef(new THREE.Clock());
  const neuralLinesRef = useRef<THREE.Line[]>([]);
  const particlesRef = useRef<THREE.Points>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // T018, T019: Initialize Three.js scene, camera, renderer
    const initScene = () => {
      // Create scene
      sceneRef.current = new THREE.Scene();
      sceneRef.current.background = null; // Transparent background

      // T019: Create camera with z-axis distance
      const aspectRatio = width
        ? width / height
        : containerRef.current!.clientWidth / height;

      cameraRef.current = new THREE.PerspectiveCamera(
        75, // field of view
        aspectRatio,
        0.1, // near clipping plane
        1000 // far clipping plane
      );
      cameraRef.current.position.z = cameraDistance;

      // Create renderer
      rendererRef.current = new THREE.WebGLRenderer({
        antialias,
        alpha: true, // transparent background
      });

      const canvasWidth = width || containerRef.current!.clientWidth;
      rendererRef.current.setSize(canvasWidth, height);
      rendererRef.current.setPixelRatio(window.devicePixelRatio);
      rendererRef.current.setClearColor(0x000000, 0); // Transparent background
      containerRef.current!.appendChild(rendererRef.current.domElement);

      // Create main group for the humanoid
      const group = new THREE.Group();
      groupRef.current = group;

      // Create humanoid figure with detailed parts
      const humanoidGroup = new THREE.Group();

      // Head (sphere)
      const headGeometry = new THREE.SphereGeometry(0.3, 16, 16);
      const headMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const head = new THREE.Mesh(headGeometry, headMaterial);
      head.position.y = 1.8;
      humanoidGroup.add(head);

      // Antenna (line on top of head)
      const antennaGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 2.1, 0),
        new THREE.Vector3(0, 2.5, 0)
      ]);
      const antennaMaterial = new THREE.LineBasicMaterial({
        color: new THREE.Color(edgeColorEnd),
      });
      const antenna = new THREE.Line(antennaGeometry, antennaMaterial);
      humanoidGroup.add(antenna);

      // Torso (cylinder)
      const torsoGeometry = new THREE.CylinderGeometry(0.4, 0.3, 1.2, 16);
      const torsoMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const torso = new THREE.Mesh(torsoGeometry, torsoMaterial);
      torso.position.y = 0.6;
      humanoidGroup.add(torso);

      // Left Arm
      const leftUpperArmGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.8, 8);
      const leftUpperArmMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const leftUpperArm = new THREE.Mesh(leftUpperArmGeometry, leftUpperArmMaterial);
      leftUpperArm.position.set(-0.7, 1.2, 0);
      leftUpperArm.rotation.z = Math.PI / 2;
      humanoidGroup.add(leftUpperArm);

      const leftLowerArmGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.8, 8);
      const leftLowerArmMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const leftLowerArm = new THREE.Mesh(leftLowerArmGeometry, leftLowerArmMaterial);
      leftLowerArm.position.set(-1.3, 1.2, 0);
      leftLowerArm.rotation.z = Math.PI / 2;
      humanoidGroup.add(leftLowerArm);

      // Right Arm
      const rightUpperArmGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.8, 8);
      const rightUpperArmMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const rightUpperArm = new THREE.Mesh(rightUpperArmGeometry, rightUpperArmMaterial);
      rightUpperArm.position.set(0.7, 1.2, 0);
      rightUpperArm.rotation.z = -Math.PI / 2;
      humanoidGroup.add(rightUpperArm);

      const rightLowerArmGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.8, 8);
      const rightLowerArmMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const rightLowerArm = new THREE.Mesh(rightLowerArmGeometry, rightLowerArmMaterial);
      rightLowerArm.position.set(1.3, 1.2, 0);
      rightLowerArm.rotation.z = -Math.PI / 2;
      humanoidGroup.add(rightLowerArm);

      // Left Leg
      const leftUpperLegGeometry = new THREE.CylinderGeometry(0.12, 0.12, 0.9, 8);
      const leftUpperLegMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const leftUpperLeg = new THREE.Mesh(leftUpperLegGeometry, leftUpperLegMaterial);
      leftUpperLeg.position.set(-0.3, -0.4, 0);
      leftUpperLeg.rotation.z = Math.PI / 2;
      humanoidGroup.add(leftUpperLeg);

      const leftLowerLegGeometry = new THREE.CylinderGeometry(0.12, 0.12, 0.9, 8);
      const leftLowerLegMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const leftLowerLeg = new THREE.Mesh(leftLowerLegGeometry, leftLowerLegMaterial);
      leftLowerLeg.position.set(-0.3, -1.1, 0);
      leftLowerLeg.rotation.z = Math.PI / 2;
      humanoidGroup.add(leftLowerLeg);

      // Right Leg
      const rightUpperLegGeometry = new THREE.CylinderGeometry(0.12, 0.12, 0.9, 8);
      const rightUpperLegMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const rightUpperLeg = new THREE.Mesh(rightUpperLegGeometry, rightUpperLegMaterial);
      rightUpperLeg.position.set(0.3, -0.4, 0);
      rightUpperLeg.rotation.z = Math.PI / 2;
      humanoidGroup.add(rightUpperLeg);

      const rightLowerLegGeometry = new THREE.CylinderGeometry(0.12, 0.12, 0.9, 8);
      const rightLowerLegMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color(edgeColorStart),
        wireframe,
      });
      const rightLowerLeg = new THREE.Mesh(rightLowerLegGeometry, rightLowerLegMaterial);
      rightLowerLeg.position.set(0.3, -1.1, 0);
      rightLowerLeg.rotation.z = Math.PI / 2;
      humanoidGroup.add(rightLowerLeg);

      // Fingers (simplified as small cylinders)
      // Left hand fingers
      for (let i = 0; i < 4; i++) {
        const fingerGeometry = new THREE.CylinderGeometry(0.03, 0.03, 0.2, 6);
        const fingerMaterial = new THREE.MeshBasicMaterial({
          color: new THREE.Color(edgeColorStart),
          wireframe,
        });
        const finger = new THREE.Mesh(fingerGeometry, fingerMaterial);
        finger.position.set(-1.45, 1.1 - (i * 0.1), 0);
        finger.rotation.z = Math.PI / 2;
        humanoidGroup.add(finger);
      }

      // Right hand fingers
      for (let i = 0; i < 4; i++) {
        const fingerGeometry = new THREE.CylinderGeometry(0.03, 0.03, 0.2, 6);
        const fingerMaterial = new THREE.MeshBasicMaterial({
          color: new THREE.Color(edgeColorStart),
          wireframe,
        });
        const finger = new THREE.Mesh(fingerGeometry, fingerMaterial);
        finger.position.set(1.45, 1.1 - (i * 0.1), 0);
        finger.rotation.z = -Math.PI / 2;
        humanoidGroup.add(finger);
      }

      group.add(humanoidGroup);
      sceneRef.current.add(group);

      // Create neural network lines (30+ lines radiating from head)
      neuralLinesRef.current = [];
      for (let i = 0; i < 36; i++) {
        const angle = (i / 36) * Math.PI * 2;
        const length = 0.5 + Math.random() * 1.0;

        const points = [
          new THREE.Vector3(0, 1.8, 0), // Start from head
          new THREE.Vector3(
            Math.cos(angle) * length,
            1.8 + Math.sin(angle * 0.5) * length * 0.5,
            Math.sin(angle) * length
          )
        ];

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
          color: new THREE.Color(edgeColorEnd),
          transparent: true,
          opacity: 0.6,
        });

        const line = new THREE.Line(geometry, material);
        neuralLinesRef.current.push(line);
        sceneRef.current.add(line);
      }

      // Create particle system (150 particles forming robot silhouette)
      const particleCount = 150;
      const particlePositions = new Float32Array(particleCount * 3);

      for (let i = 0; i < particleCount; i++) {
        // Position particles around the humanoid shape
        const i3 = i * 3;

        // Create positions that follow a humanoid silhouette
        const part = Math.floor(Math.random() * 5); // 0: head, 1: torso, 2: arms, 3: legs, 4: around

        switch(part) {
          case 0: // Head area
            particlePositions[i3] = (Math.random() - 0.5) * 0.8; // x
            particlePositions[i3 + 1] = 1.5 + Math.random() * 0.6; // y
            particlePositions[i3 + 2] = (Math.random() - 0.5) * 0.8; // z
            break;
          case 1: // Torso area
            particlePositions[i3] = (Math.random() - 0.5) * 1.0; // x
            particlePositions[i3 + 1] = 0.3 + (Math.random() - 0.5) * 1.0; // y
            particlePositions[i3 + 2] = (Math.random() - 0.5) * 0.8; // z
            break;
          case 2: // Arms area
            const armSide = Math.random() > 0.5 ? 1 : -1;
            particlePositions[i3] = armSide * (0.8 + Math.random() * 0.6); // x - left or right arm
            particlePositions[i3 + 1] = 1.0 + (Math.random() - 0.5) * 0.5; // y
            particlePositions[i3 + 2] = (Math.random() - 0.5) * 0.5; // z
            break;
          case 3: // Legs area
            const legSide = Math.random() > 0.5 ? 1 : -1;
            particlePositions[i3] = legSide * (0.2 + Math.random() * 0.2); // x - left or right leg
            particlePositions[i3 + 1] = -0.5 + Math.random() * (-1.2); // y
            particlePositions[i3 + 2] = (Math.random() - 0.5) * 0.4; // z
            break;
          default: // Around the figure
            const radius = 1.0 + Math.random() * 1.5;
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.random() * Math.PI;

            particlePositions[i3] = radius * Math.sin(phi) * Math.cos(theta); // x
            particlePositions[i3 + 1] = 0.5 + radius * Math.cos(phi); // y
            particlePositions[i3 + 2] = radius * Math.sin(phi) * Math.sin(theta); // z
            break;
        }
      }

      const particleGeometry = new THREE.BufferGeometry();
      particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));

      const particleMaterial = new THREE.PointsMaterial({
        color: new THREE.Color(edgeColorEnd),
        size: 0.05,
        transparent: true,
        opacity: 0.8,
      });

      const particles = new THREE.Points(particleGeometry, particleMaterial);
      particlesRef.current = particles;
      sceneRef.current.add(particles);
    };

    // T021: Animation loop with requestAnimationFrame
    const animate = () => {
      if (!groupRef.current || !sceneRef.current || !cameraRef.current || !rendererRef.current) {
        return;
      }

      const delta = clockRef.current.getDelta();
      const elapsedTime = clockRef.current.getElapsedTime();

      // Rotate the entire humanoid with walking motion
      groupRef.current.rotation.y += rotationSpeed;

      // Walking cycle animation - alternate arm and leg movements
      const walkCycle = Math.sin(elapsedTime * 3) * 0.3; // 3 Hz walking motion

      // Animate left arm (opposite to left leg)
      const leftUpperArm = groupRef.current.children[0].children[4] as THREE.Mesh;
      const leftLowerArm = groupRef.current.children[0].children[5] as THREE.Mesh;
      if (leftUpperArm) leftUpperArm.rotation.z = Math.PI / 2 + walkCycle;
      if (leftLowerArm) leftLowerArm.rotation.z = Math.PI / 2 + walkCycle * 0.7;

      // Animate right arm (opposite to right leg)
      const rightUpperArm = groupRef.current.children[0].children[6] as THREE.Mesh;
      const rightLowerArm = groupRef.current.children[0].children[7] as THREE.Mesh;
      if (rightUpperArm) rightUpperArm.rotation.z = -Math.PI / 2 - walkCycle;
      if (rightLowerArm) rightLowerArm.rotation.z = -Math.PI / 2 - walkCycle * 0.7;

      // Animate left leg
      const leftUpperLeg = groupRef.current.children[0].children[8] as THREE.Mesh;
      const leftLowerLeg = groupRef.current.children[0].children[9] as THREE.Mesh;
      if (leftUpperLeg) leftUpperLeg.rotation.z = Math.PI / 2 + walkCycle * 1.2;
      if (leftLowerLeg) leftLowerLeg.rotation.z = Math.PI / 2 + walkCycle * 0.8;

      // Animate right leg
      const rightUpperLeg = groupRef.current.children[0].children[10] as THREE.Mesh;
      const rightLowerLeg = groupRef.current.children[0].children[11] as THREE.Mesh;
      if (rightUpperLeg) rightUpperLeg.rotation.z = Math.PI / 2 - walkCycle * 1.2;
      if (rightLowerLeg) rightLowerLeg.rotation.z = Math.PI / 2 - walkCycle * 0.8;

      // Pulsing animation for neural lines
      neuralLinesRef.current.forEach((line, index) => {
        const pulse = Math.sin(elapsedTime * Math.PI) * 0.5 + 0.5; // 2 second rhythm
        const baseOpacity = 0.6;
        line.material.opacity = baseOpacity * pulse;
      });

      // Gentle floating motion for particles
      if (particlesRef.current) {
        const positions = particlesRef.current.geometry.attributes.position.array;
        for (let i = 0; i < positions.length; i += 3) {
          // Add a gentle floating motion based on time and position
          positions[i + 1] += Math.sin(elapsedTime + positions[i] * 0.1) * 0.005;
        }
        particlesRef.current.geometry.attributes.position.needsUpdate = true;
      }

      // Render scene
      rendererRef.current.render(sceneRef.current, cameraRef.current);

      // Continue animation loop
      animationFrameIdRef.current = requestAnimationFrame(animate);
    };

    // T023: Handle window resize events
    const handleResize = () => {
      if (!cameraRef.current || !rendererRef.current || !containerRef.current) {
        return;
      }

      const canvasWidth = width || containerRef.current.clientWidth;
      const aspectRatio = canvasWidth / height;

      // Update camera aspect ratio
      cameraRef.current.aspect = aspectRatio;
      cameraRef.current.updateProjectionMatrix();

      // Update renderer size
      rendererRef.current.setSize(canvasWidth, height);
    };

    // Initialize scene and start animation
    initScene();
    animate();

    // Add resize listener
    window.addEventListener('resize', handleResize);

    // T022: Cleanup function
    return () => {
      // Cancel animation frame
      if (animationFrameIdRef.current !== null) {
        cancelAnimationFrame(animationFrameIdRef.current);
      }

      // Remove resize listener
      window.removeEventListener('resize', handleResize);

      // Dispose renderer
      if (rendererRef.current) {
        rendererRef.current.dispose();

        // Remove canvas from DOM
        if (rendererRef.current.domElement && containerRef.current) {
          containerRef.current.removeChild(rendererRef.current.domElement);
        }
      }

      // Dispose all geometries and materials
      if (sceneRef.current) {
        sceneRef.current.traverse((object) => {
          if (object instanceof THREE.Mesh) {
            if (object.geometry) object.geometry.dispose();
            if (object.material) {
              if (Array.isArray(object.material)) {
                object.material.forEach(material => material.dispose());
              } else {
                object.material.dispose();
              }
            }
          } else if (object instanceof THREE.Line) {
            if (object.geometry) object.geometry.dispose();
            if (object.material) {
              if (Array.isArray(object.material)) {
                object.material.forEach(material => material.dispose());
              } else {
                object.material.dispose();
              }
            }
          } else if (object instanceof THREE.Points) {
            if (object.geometry) object.geometry.dispose();
            if (object.material) object.material.dispose();
          }
        });
      }

      // Clear refs
      sceneRef.current = null;
      cameraRef.current = null;
      rendererRef.current = null;
      groupRef.current = null;
      animationFrameIdRef.current = null;
      neuralLinesRef.current = [];
      particlesRef.current = null;
    };
  }, [width, height, rotationSpeed, wireframe, geometryType, edgeColorStart, edgeColorEnd, antialias, cameraDistance]);

  return (
    <div
      ref={containerRef}
      style={{
        width: width ? `${width}px` : '100%',
        height: `${height}px`,
        margin: '0 auto',
        minHeight: '600px', // Ensure minimum 600px as requested
      }}
    />
  );
};

export default ThreeScene;
