/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      colors: {
        'cyan-neon': '#00F0FF',
        'magenta-neon': '#FF2A6D',
        'dark-bg': '#0A0A0F',
        'light-bg': '#F8F9FB',
        'card-dark': '#12131A',
        'text-dark': '#D6D6D6',
        'text-light': '#2A2A2A'
      },
      fontFamily: {
        inter: ['Inter', 'sans-serif'],
        poppins: ['Poppins', 'sans-serif']
      }
    }
  },
  plugins: []
};
