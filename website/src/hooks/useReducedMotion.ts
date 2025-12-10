import { useMediaQuery } from './useMediaQuery';

/**
 * Custom hook to detect user's reduced motion preference
 * Respects accessibility settings for users who prefer reduced motion
 * @returns boolean indicating if user prefers reduced motion
 *
 * @example
 * const prefersReducedMotion = useReducedMotion();
 * if (!prefersReducedMotion) {
 *   // Enable animations
 * }
 */
export function useReducedMotion(): boolean {
  return useMediaQuery('(prefers-reduced-motion: reduce)');
}
