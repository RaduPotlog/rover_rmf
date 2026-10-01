/* Mechatronics Academy colours: the logo's orange on a near-black app bar. */
import { createTheme } from '@mui/material';

const ORANGE = '#ffaa00';
const BAR = '#1d1d1b';

export const roverTheme = createTheme({
  palette: {
    primary: { main: ORANGE, contrastText: '#000' },
    secondary: { main: ORANGE, contrastText: '#000' },
  },
  components: {
    MuiAppBar: {
      styleOverrides: {
        // The app bar takes primary by default; keep it dark so the round logo stands out.
        root: { backgroundColor: BAR, color: '#fff' },
      },
    },
  },
});
