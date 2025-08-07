import React from 'react';
import { Paper, Typography, Button, ButtonGroup, Box } from '@mui/material';
import { PlayArrow, Pause, Replay } from '@mui/icons-material';

const ControlPanel = () => {
    return (
        <Paper sx={{ p: 2 }} elevation={3}>
            <Typography variant="h6" gutterBottom>Control Panel</Typography>
            <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2, mt: 2 }}>
                <Typography variant="body2">Path Control</Typography>
                <ButtonGroup variant="contained" fullWidth>
                    <Button color="primary" startIcon={<PlayArrow />}>Start</Button>
                    <Button color="secondary" startIcon={<Pause />}>Pause</Button>
                    <Button color="error" startIcon={<Replay />}>Reset</Button>
                </ButtonGroup>
                <Typography variant="body2">System Actions</Typography>
                <Button variant="outlined" color="warning">Emergency Stop</Button>
            </Box>
        </Paper>
    );
};

export default ControlPanel;
