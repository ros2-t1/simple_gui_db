import React from 'react';
import { Box, Typography, CssBaseline, ThemeProvider, createTheme } from '@mui/material';
import MapDisplay from './MapDisplay';
import ControlPanel from './ControlPanel';
import RobotStatus from './RobotStatus';

const darkTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: '#1abc9c',
    },
    background: {
      default: '#2c3e50',
      paper: '#34495e',
    },
  },
  typography: {
    fontFamily: '"Segoe UI", Tahoma, Geneva, Verdana, sans-serif',
    h4: {
      fontWeight: 600,
    },
  },
});

function App() {
  return (
    <ThemeProvider theme={darkTheme}>
      <CssBaseline />
      <Box sx={{ display: 'flex', flexDirection: 'column', height: '100vh', p: 2 }}>
        <Typography variant="h4" gutterBottom component="div" sx={{ textAlign: 'center', flexShrink: 0, mb: 2 }}>
          Robot Fleet Control Dashboard
        </Typography>
        <Box sx={{ display: 'flex', flexGrow: 1, gap: 2, overflow: 'hidden' }}>
          <Box sx={{ flex: '3 1 0', minWidth: 0, height: '100%' }}>
            <MapDisplay />
          </Box>
          <Box sx={{ flex: '1 1 0', display: 'flex', flexDirection: 'column', gap: 2, minWidth: 0 }}>
            <RobotStatus />
            <ControlPanel />
          </Box>
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export default App;
